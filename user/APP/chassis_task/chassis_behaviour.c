/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      Complete the chassis behavior task.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "delay.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"


/**
  * @brief          Under the weak state of the chassis, the chassis mode is raw, so the
                    set value will be sent directly to the can bus, so the set value is set to 0.
  * @author         RM
  * @param[in]      Vx_set forward speed set value will be sent directly to the can bus
  * @param[in]      The speed setpoint around vy_set will be sent directly to the can bus.
  * @param[in]      The speed of the wz_set rotation will be sent directly to the can bus.
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          Under the behavior state machine where the chassis does not move, the chassis
                    mode does not follow the angle.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set rotation speed, the rotation speed is the chassis angular speed of the control chassis
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          The chassis follows the behavior of the pan/tilt. The chassis mode follows the pan/tilt angle.
                    The chassis rotation speed calculates the angular velocity of the chassis rotation based on
                    the angle difference.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      The angle between the angle_set chassis and the pan/tilt
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          The chassis follows the behavior of the chassis yaw, the chassis mode is to follow the chassis angle,
                    the chassis rotation speed will calculate the angular velocity of the chassis rotation according to
                    the angle difference
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Angle_set chassis setting yaw, range -PI to PI
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          The chassis does not follow the angle of the behavior state machine, the chassis mode is not
                    following the angle, the chassis rotation speed is directly set by the parameters
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          Under the behavior of the chassis open loop, the chassis mode is the raw native state,
                    so the set value will be sent directly to the can bus.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          Do a swirly periodic pivot
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */
static void chassis_swirl_motion(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//Chassis behavior state machine
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //Remote control setting behavior mode
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_SWIRL;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }

    //When the pan/tilt enters certain states, the chassis remains stationary.
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }

    //Select chassis state machine based on behavioral state machine
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //When the behavior is that the chassis is weak,
                                                              //set the chassis state machine to raw, native state machine.
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //When the behavior is that the chassis does not move,
                                                                        //set the chassis state machine to the chassis without
                                                                        //following the angle state machine.
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //When the behavior is that the normal infantry
                                                                            //follows the pan/tilt, the chassis state machine
                                                                            //is set to follow the pan/tilt angle state machine for the chassis.
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //When the behavior is that the engineering follows the
                                                                             //chassis angle, the chassis state machine is set to the
                                                                             //chassis following the chassis angle state machine.
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //When the behavior is that the chassis does not follow the angle,
                                                                        //then the chassis state machine is set to the chassis without
                                                                        //following the angle state machine.
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //When the behavior is the chassis open loop, set the chassis state machine
                                                              //to the chassis native raw state machine.
    }
    else if (chassis_behaviour_mode == CHASSIS_SWIRL)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SWIRL; //When the behavior is the chassis open loop, set the chassis state machine
                                                                        //to the chassis native raw state machine.
    }
}
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_SWIRL)
    {
        chassis_swirl_motion(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}
/**
  * @brief          Under the weak state of the chassis, the chassis mode is raw, so the set value will be sent
                    directly to the can bus, so the set value is set to 0.
  * @author         RM
  * @param[in]      Vx_set forward speed set value will be sent directly to the can bus
  * @param[in]      The speed setpoint around vy_set will be sent directly to the can bus.
  * @param[in]      The speed of the wz_set rotation will be sent directly to the can bus.
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f; // The f just explicitly casts it as a floating point number
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
/**
  * @brief          Under the behavior state machine where the chassis does not move, the chassis mode does not
                    follow the angle.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set (moves sideways)
  * @param[in]      Wz_set rotation speed, the rotation speed is the chassis angular speed of the control chassis
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          The chassis follows the behavior of the pan/tilt. The chassis mode follows the pan/tilt angle. The chassis rotation
                    speed calculates the angular velocity of the chassis rotation based on the angle difference.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      The angle between the angle_set chassis and the pan/tilt
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //The swing angle is generated using the sin function, and the swing_time is the input value of the sin function.
    static fp32 swing_time = 0.0f;
    //Swing_time is the calculated angle
    static fp32 swing_angle = 0.0f;
    //Max_angle is the magnitude of the sin function
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //Add_time is the speed at which the swing angle changes, the faster the maximum
    static fp32 const add_time = PI / 250.0f;
    //Enable swing flag
    static uint8_t swing_flag = 0;

    //Calculate the original input signal of the remote control

    //Determine if you want to swing
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //Determine if the keyboard input is in control of the chassis movement, and the chassis reduces the swing angle during motion.
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    //Sin function generates control angle
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //The sin function does not exceed 2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}

/**
  * @brief          The chassis follows the behavior of the chassis yaw, the chassis mode is to follow the chassis angle,
                    the chassis rotation speed will calculate the angular velocity of the chassis rotation according
                    to the angle difference
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Angle_set chassis setting yaw, range -PI to PI
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          The chassis does not follow the angle of the behavior state machine, the chassis mode is
                    not following the angle, the chassis rotation speed is directly set by the parameters
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          Under the behavior of the chassis open loop, the chassis mode is the raw native state,
                    so the set value will be sent directly to the can bus.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

/**
  * @brief          Under the behavior of the chassis open loop, the chassis mode is the raw native state,
                    so the set value will be sent directly to the can bus.
  * @author         RM
  * @param[in]      Vx_set forward speed
  * @param[in]      Velocity around vy_set
  * @param[in]      Wz_set chassis set rotation speed
  * @param[in]      Chassis_move_rc_to_vector chassis data
  * @retval         Return empty
  */
 static void chassis_swirl_motion(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
 {
     if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
     {
         return;
     }

     *vx_set = 0.0f;
     *vy_set = 0.0f;
     *wz_set = 0.0f;
 }
