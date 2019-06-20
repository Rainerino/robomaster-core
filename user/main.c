/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32³õÊ¼»¯ÒÔ¼°¿ªÊ¼ÈÎÎñfreeRTOS¡£hÎÄ¼þ¶¨ÒåÏà¹ØÈ«¾Öºê¶¨ÒåÒÔ¼°
  *             typedef Ò»Ð©³£ÓÃÊý¾ÝÀàÐÍ
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
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"

void BSP_init(void);

int main(void)
{
    BSP_init();
    delay_ms(100);
    startTast();
    vTaskStartScheduler();
		
    while (1)
    {
        ;
    }
}



//ËÄ¸ö24v Êä³ö ÒÀ´Î¿ªÆô ¼ä¸ô 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

extern void USART_setup(void);

void BSP_init(void)
{
    //ÖÐ¶Ï×é 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //³õÊ¼»¯µÎ´ðÊ±ÖÓ
    delay_init(configTICK_RATE_HZ);
    //Á÷Ë®µÆ£¬ºìÂÌµÆ³õÊ¼»¯
    led_configuration();
    //stm32 °åÔØÎÂ¶È´«¸ÐÆ÷³õÊ¼»¯
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 Ëæ»úÊý·¢ÉúÆ÷³õÊ¼»¯
    RNG_init();
#endif
    //24Êä³ö¿ØÖÆ¿Ú ³õÊ¼»¯
    power_ctrl_configuration();
    //Ä¦²ÁÂÖµç»úPWM³õÊ¼»¯
    fric_PWM_configuration();
    //·äÃùÆ÷³õÊ¼»¯
    buzzer_init(30000, 90);
    //¼¤¹âIO³õÊ¼»¯
    laser_configuration();
    //¶¨Ê±Æ÷6 ³õÊ¼»¯
    TIM6_Init(60000, 90);
    //CAN½Ó¿Ú³õÊ¼»¯
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v Êä³ö ÒÀ´ÎÉÏµç
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //Ò£¿ØÆ÷³õÊ¼»¯
    remote_control_init();
    //flash¶ÁÈ¡º¯Êý£¬°ÑÐ£×¼Öµ·Å»Ø¶ÔÓ¦²ÎÊý
    cali_param_init();
		
		  USART_setup();
}
