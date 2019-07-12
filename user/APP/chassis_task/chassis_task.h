/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
// Ratio between m3508 motor to chassis, two defines in case of changing motors
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�  max forward speed
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�  max L/R speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
// scaling front and back wheel speed for turning; 0 means center of mass is also the center of rotation and no compensation necessary
#define CHASSIS_WZ_SET_SCALE 0.1f

//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,
  CHASSIS_VECTOR_RAW,
  CHASSIS_VECTOR_SWIRL,

  //  CHASSIS_AUTO,
  //  CHASSIS_FOLLOW_YAW,
  //  CHASSIS_ENCODER,
  //  CHASSIS_NO_ACTION,
  //  CHASSIS_RELAX,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} Chassis_Motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��   		pointer to the RC used
  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  Chassis_Motor_t motor_chassis[4];          //���̵������
  PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
  PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s  			chassis speed: forward as positive, unit m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s  			chassis speed: left as positive, unit m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s  	chassis rotational speed: counter clockwise as positive, unit rad/s
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s  		set forward/back speed
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s  		set left/right speed
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s  set chassis rotational speed
  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s  					angle between chassis and gimbal, unit rad/s
  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s					front / back max speed, in m/s 
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s					front / back min speed, in m/s 
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s					left / right max speed, in m/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s					left / right min speed, in m/s
  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�  		yaw angle based on gyro and gimbal motors
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�		pitch angle based on gyro and gimbal motors
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�		roll angle based on gyro and gimbal motors

} chassis_move_t;

extern void chassis_task(void *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
