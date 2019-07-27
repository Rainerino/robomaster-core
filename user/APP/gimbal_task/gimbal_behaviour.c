/**
  * @file       gimbal_behaviour.c
  * @brief      Controls gimbal based on 4 modes:
  *             - Gyro control mode, where control is based on the angles provided by gyro (between -π and π)
  *             - Encoder mode, where control is based on encoder feedback of the motors
  *             - Calibration mode
  *             - Stopped mode
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-16-2019     Pedram          1. Reorganized code
*/

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "Detect_Task.h"
#include "stm32f4xx.h"
#include "user_lib.h"
#include "stdio.h"
#include "stdlib.h"

/**
  * @brief      Turns buzzer on during Gimbal calibration
  * @author     RM
  */
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)

/**
  * @brief      Turns buzzer off during Gimbal calibration
  * @author     RM
  */
#define gimbal_warn_buzzer_off() buzzer_off()

/**
  * @brief      Takes the absolute value of a number
  * @author     RM
  * @param[in]  x, the number
  * @retval     abs(x)
  */
#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
  * @brief      Handles cases when RC joystick is not quite centered
  * @author     RM
  * @param[in]  input, RC joystick value
  * @param[in]  output, control value after deadline is applied
  * @param[in]  deadline, specifies range in which joystick is considered centered
  */
#define rc_deadline_limit(input, output, deadline) \
	{                                              \
		if (input > deadline || input < -deadline) \
            output = input;                        \
        else                                       \
            output = 0;                            \
    }
    
/**
  * @brief      Calibrate gimbal: determine if gimbal is at end positions by measuring angular velocity
  * @author     RM
  * @param[in]  gyro, gyro angular velocity reading (rad/s)
  * @param[in]  cmd_time, timer that sets to zero when it reaches GIMBAL_CALI_STEP_TIME
  * @param[in]  angle_set, gyro angle recorded (rad)
  * @param[in]  angle, feedback angle (rad)
  * @param[in]  ecd_set, encoder recorded (raw)
  * @param[in]  ecd, feedback encoder reading (raw)
  * @param[in]  step, counter
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if (gyro < GIMBAL_CALI_GYRO_LIMIT)                                           \
        {                                                                            \
            cmd_time++;                                                              \
            if (cmd_time > GIMBAL_CALI_STEP_TIME)                                    \
            {                                                                        \
                cmd_time = 0;                                                        \
                angle_set = angle;                                                   \
                ecd_set = ecd;                                                       \
                step++;                                                              \
            }                                                                        \
        }                                                                            \
    }

static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);

/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，使得云台无力
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
		
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

// Position based control
static void gimbal_position_based_control (fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

// Position based control from USART transmission
static void gimbal_pos_USART (fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief      Detect a keydown event
  * @author     The Jingler
	* @param[in]  key, keyboard key to look for a keydown event
  * @param[in]  now, current state of keyboard key
  * @param[in]  past, state of keyboard key from previous loop
  * @retval     1 (True) if keydown event occurs, 0 (False) otherwise
  */
#define keydown(key, now, past) ( (now & key) && !(past & key) )


//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          云台行为状态机以及电机状态机设置
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
	
gimbal_motor_mode_e test = GIMBAL_MOTOR_ENCONDE;
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		else if (gimbal_behaviour == GIMBAL_POSITION_BASED)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		else if (gimbal_behaviour == GIMBAL_POS_USART)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
		else if (gimbal_behaviour == GIMBAL_POSITION_BASED)
		{
				gimbal_position_based_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
		}
		else if (gimbal_behaviour == GIMBAL_POS_USART)
		{
				gimbal_pos_USART(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
		}
		
    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI) 
		//|| gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          云台在某些行为下，需要射击停止
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
  * @brief          云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //校准行为，return 不会设置其他的模式
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }
    //如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //初始化模式判断是否到达中值位置
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        //到达中值 计时
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            //到达初始化位置
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            //没有到达初始化位置，时间计时
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) && !toe_is_error(DBUSTOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }
		
		static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;	// from line 401
		static int c_key_pressed = 0;
		static int v_key_pressed = 0;
		
		//开关控制 云台状态
		if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
		{
				gimbal_behaviour = GIMBAL_POS_USART;
		}
		else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
		{
				gimbal_behaviour = GIMBAL_ZERO_FORCE; //NO FORCE
		}
		else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
		{ 
				gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
		}

		
		if(keydown(CHANGE_MODE_TO_CONTROL_KEY, gimbal_mode_set->gimbal_rc_ctrl->key.v, last_gimbal_behaviour))
		{
				c_key_pressed = 1;
				v_key_pressed = 0;
		}
		else if(keydown(CHANGE_MODE_TO_USART_KEY, gimbal_mode_set->gimbal_rc_ctrl->key.v, last_gimbal_behaviour))
		{
				v_key_pressed = 1;
				c_key_pressed = 0;
		}
		
		if(c_key_pressed)
		{
			gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
		}
		else if(v_key_pressed)
		{
			gimbal_behaviour = GIMBAL_POS_USART;
		}

    if( toe_is_error(DBUSTOE))
    {

        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    //判断进入init状态机
    {
        // static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;		// to line 377
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
				// Ensures the gimbal initialized to mid before entering postion-based setting
				else if (last_gimbal_behaviour != GIMBAL_POSITION_BASED && last_gimbal_behaviour != GIMBAL_INIT && gimbal_behaviour == GIMBAL_POSITION_BASED)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

    static uint16_t motionless_time = 0;
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        //遥控器 键盘均无输入，进入motionless状态
        if (int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[0]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[1]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[2]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[3]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.x) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.y) < GIMBAL_MOTIONLESS_RC_DEADLINE && gimbal_mode_set->gimbal_rc_ctrl->key.v == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r == 0)
        {
            if (motionless_time < GIMBAL_MOTIONLESS_TIME_MAX)
            {
                motionless_time++;
            }
        }
        else
        {
            motionless_time = 0;
        }

        if (motionless_time == GIMBAL_MOTIONLESS_TIME_MAX)
        {
            gimbal_behaviour = GIMBAL_MOTIONLESS;
        }
    }
    else
    {
        motionless_time = 0;
    }


}

// Position based control on joystick
static void gimbal_position_based_control (fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    {
        static fp32 yaw_target_angle, pitch_target_angle;
			
				// Take reading from RC and convert bewteen channel reading and yaw angle
				yaw_target_angle = gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel] * Yaw_RC_SCALE; 
				// Take reading from RC and convert bewteen channel reading and pitch angle
				pitch_target_angle = gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel] * Pitch_RC_SCALE; 
				
				// set yaw and pitch to the designated angles
				*yaw = (yaw_target_angle - gimbal_control_set->gimbal_yaw_motor.relative_angle) * PositionSpeed; 
				*pitch = (pitch_target_angle - gimbal_control_set->gimbal_pitch_motor.relative_angle) * PositionSpeed;
			
    }
}

volatile uint8_t USART_Data = 0;
volatile int Data_received = 0;
extern void USART_puts(USART_TypeDef *USARTx, volatile char *str);
extern void USART_cmd_shoot(void);

// Position based control from USART
static void gimbal_pos_USART (fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
			return;
    }

    {
			double yaw_change_intensity = gimbal_control_set->gimbal_yaw_motor.relative_angle;
			double pitch_change_intensity = gimbal_control_set->gimbal_pitch_motor.relative_angle;
			static int i = 0;
			
			const int N = 4;	// total number of positions
			const int YAW = 0, PITCH = 1;
			double aim_to[N][2] =
				{ /* yaw   pit												 _.-0-._			*/
						{0.0,  0.1},	/* pos 0						/		|	 	\			*/
						{0.1,  0.0},	/* pos 1					 3----+----1		*/
						{0.0, -0.1},	/* pos 2						\		| 	/			*/
						{-0.1, 0.0}		/* pos 3						 `-.2.-`			*/
				};

			
			if(Data_received == 1)
			{
					++i;
					
					if(i < 1000)	// aim long enough to shoot
					{
							yaw_change_intensity = aim_to[USART_Data][YAW];
							pitch_change_intensity = aim_to[USART_Data][PITCH];
							
							if(i == 800)	// when to shoot
							{
								USART_cmd_shoot();
							}
					}
					else	// reset		
					{
							Data_received = 0;
							i = 0;
					}
			}
			
			/*
			// |x|x|x|x|x|x|x|, data received from USART				0b1010011 & 0001111
			// |--p--|---y---|, p = pitch, y = yaw
			// 0 <= p <= 6, 0 <= y <= 14
			
			// 55 corresponds to p=3, y=7
			// yaw/pitch_change intensity are two measures between -1 and 1 
			// for how fast (and direction) pitch/yaw should change to aim
			if (USART_Data != 55) {
				if (0 <= USART_Data%16 && USART_Data%16 <= 14)
					yaw_change_intensity = (USART_Data % 16 - 7)/7.0;
				if (0 <= USART_Data/16 && USART_Data/16 <= 6)
					pitch_change_intensity = -(USART_Data / 16 - 3)/3.0;
				USART_Data = 55;
			}
			*/
			
			/*
			double pitch_change_intensity = 0.0;
			double yaw_change_intensity = 0.0;
			static int i = 0;
			
			if(Data_received == 1)
			{
					++i;
					if(i < 75)	// delay before aim
					{
							yaw_change_intensity = 0.0f;
							pitch_change_intensity = 0.0f;
					}
					else if(i < 1250)	// aim long enough to shoot
					{
							yaw_change_intensity = USART_Data / -100;	// position
							pitch_change_intensity = 0; //-0.05f;
							// shoot but only once
					}
					else	// reset		
					{
							yaw_change_intensity = 0.0f;
							pitch_change_intensity = 0.0f;
							Data_received = 0;
							i = 0;
					}
			}
			*/
			
			
			/*
			{
			double yaw_change_intensity = 0.0;		// -1 <= yaw_change_intensity <= 1
			double pitch_change_intensity = 0.0;	// -1 <= pitch_change_intensity <= 1
			double USART_YAW_SCALE = 10;
			double USART_PITCH_SCALE = 10;
			double angle_rad = USART_Data / 255 * 360 * 3.1415926535 / 180;		// 255th's of one circle -> degrees -> radians
			
			// USART_puts(USART6, (char*) &angle_rad);

			yaw_change_intensity = sin(0.5) * 1;
			pitch_change_intensity = cos(0.5) * 1;
			
			//USART_puts(USART6, (char*) &yaw_change_intensity);
			//USART_puts(USART6, (char*) &pitch_change_intensity);
			
			*/
			
			/*
			*yaw = yaw_change_intensity * 0.2;
			*pitch = pitch_change_intensity * 0.2;
    }
			*/
			
			*yaw = (yaw_change_intensity - gimbal_control_set->gimbal_yaw_motor.relative_angle) * 0.005; 
			*pitch = (pitch_change_intensity - gimbal_control_set->gimbal_pitch_motor.relative_angle) * 0.005;
			
			while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);
			USART_SendData(USART6, 1);

    }
}


/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，使得云台无力
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TurnKeyBoard) && !(last_turn_keyboard & TurnKeyBoard))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //保存掉头的目标值
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //不断控制到掉头的目标值，正转，反装是随机
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TurnSpeed;
            }
            else
            {
                *yaw -= TurnSpeed;
            }
        }
        //到达pi （180°）后停止
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //不需要处理，
}
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
