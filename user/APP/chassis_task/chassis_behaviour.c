/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      Implements chassis behaviour tasks.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Complete
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

/**
  * @brief          In chassis zero force control, the chassis is in raw mode, the zero vectors will be transmitted through CANBUS directly without feedback loop.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          In chassis no move control, the chassis is sent zero velocity vectors with feedback loop. The motors will try and not move.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
	
	/**
  * @brief          In chassis follow gimbal yaw control, the chassis follws the gimbal by calculating the angular velocity from the angle differences between chassis and gimbal.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          In chassis engineer follow chassis yaw, chassis turn speed is position based, while x and y movements are normal 
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          In chassis no follow yaw control, chassis turn speed is controlled directly through RC, there is feedback loop
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */
	
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          In chassis open set control, chassis is under raw signal, and RC values will be directly sent to CANBUS without feedback loop
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */
	
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

//Chassis default behaviour
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    // Chassis behaviour mode determined by RC
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW; 
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode =  CHASSIS_NO_MOVE; 
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    // Chassis stays stationery (no move mode) when gimbal is in certain states
		// Overrides chassis behaviour mode from RC
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }

    // Chassis behaviour mode lookup table
		// Updates the chassis move mode according to chassis behaviour mode
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
				// When chassis behaviour mode is "CHASSIS_ZERO_FORCE",	 set chassis control mode to CHASSIS_VECTOR_RAW, raw control.  
				chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
				// When chassis behaviour mode is "CHASSIS NO MOVE",	 set chassis control mode to CHASSIS VECTOR NO FOLLOW YAW.
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
		}
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
				// When chassis behaviour mode is "CHASSIS INFANTRY FOLLOW GIMBAL YAW",	 set chassis control mode to CHASSIS VECTOR FOLLOW GIMBAL YAW.
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}

// Chassis behaviour mode look up table 
// Calls different chassis control function based on the value of Chassis behaviour mode.
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
}

/**
  * @brief          In chassis zero force control, the chassis is in raw mode, the zero vectors will be transmitted through CANBUS directly without feedback loop.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          In chassis no move control, the chassis is sent zero velocity vectors with feedback loop. The motors will try and not move.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
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
  * @brief          In chassis follow gimbal yaw control, the chassis follws the gimbal by calculating the angular velocity from the angle differences between chassis and gimbal.
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      angle_set: calculated turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    // Swing angle is the result of a sine function with parameter swing time
    static fp32 swing_time = 0.0f; 
    static fp32 swing_angle = 0.0f;
    //max_angle is the max parameter for sine
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time determines the rate of change of swing angles, larger add_time means faster swing
    static fp32 const add_time = PI / 250.0f;
    
    static uint8_t swing_flag = 0;


    // Determine whether swing key is pressed
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

    // Checks whether chassis is being controlled through RC, and sets the max angle accordingly
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
		
    // Use sine function to generate swing angle
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
		
    // sine parameter does not exceed 2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}

/**
  * @brief          In chassis engineer follow chassis yaw, chassis turn speed is position based, while x and y movements are normal 
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
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
  * @brief          In chassis no follow yaw control, chassis turn speed is controlled directly through RC, there is feedback loop
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
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
  * @brief          In chassis open set control, chassis is under raw signal, and RC values will be directly sent to CANBUS without feedback loop
  * @author         RM
	* @param[in]      vx_set: raw forward and backward movement
	* @param[in]      vy_set: raw left and right movement
	* @param[in]      wz_set: raw turn speed
	* @param[in]      chassis_move_rc_to_vector: data from rc
  * @retval         Return void
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
