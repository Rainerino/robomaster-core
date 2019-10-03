/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
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
void Serial_setup(void);

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

//�ĸ�24v ��� ���ο��� ��� 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ��
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    led_configuration();
    //stm32 �����¶ȴ�������ʼ��
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 �������������ʼ��
    RNG_init();
#endif
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ��
    fric_PWM_configuration();
    //��������ʼ��
    //buzzer_init(30000, 90);
    //����IO��ʼ��
    laser_configuration();
    //��ʱ��6 ��ʼ��
    TIM6_Init(60000, 90);
    //CAN�ӿڳ�ʼ��
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v ��� �����ϵ�
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //ң������ʼ��
    remote_control_init();
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    cali_param_init();
		
		Serial_setup();
}

void Serial_setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable the GPIOG clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	// Enable peripheral clock for USART6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	// Force USART6 clock to reset
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

	// Setup GPIO pins for UART

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);	// Tx
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);		// Rx

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;				// rc USART uses GPIO_PuPd_NOPULL
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	USART_DeInit(USART6);

	// Configure UART - this all needs to match the config on the Pi
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			// These are the default settings
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// All set by USART_StructInit()
	USART_InitStructure.USART_Parity = USART_Parity_No;					// These details are here for clarity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;


	// USART_StructInit(&USART_InitStructure); 	// all the settings above are the default settings, this sets them all
	USART_Init(USART6, &USART_InitStructure);

	// enable interrupts
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	// enable USART6 peripheral
	USART_Cmd(USART6, ENABLE);

	// configure NVIC for interrupts - only on Rx
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		// set priority group to highest priority achieveable by sw (hw can go negative)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					// set priority highest in subgroup
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
