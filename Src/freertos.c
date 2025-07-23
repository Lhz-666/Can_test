/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "PID.h"
#include "FS_remote_control_unit.h"
//void can_filter_init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Remote_Protected_MAX_time 100//这里需要根据实际情况调整
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int16_t GetSpeed=0,GetAngle=0,GetCurrent=0,GetTemperature=0;
int16_t Last_Get_Real_Angle=0,Get_Real_Angle=0;//CAN通信返回的值
int16_t SetVoltage=12500;//这里是电压值。电压给定值范围：-25000~0~25000, 对应最大转矩电流范围 -3A~0~3A
int16_t SetSpeed=200;//这里是转速
int16_t SetPosition=200;
int8_t number;//这里是圈数
MOTOR_t motor_6020;

uint16_t Remote_Protected_time=0;
short Remote_Protected_Flag=1;//0为不需要，1为需要

extern uint8_t rx_buffer[32];

extern uint16_t channels[32];
uint16_t last_second_channels=0;
uint16_t Get_data_times=0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId printf_dataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of printf_data */
  osThreadDef(printf_data, StartTask02, osPriorityAboveNormal, 0, 128);
  printf_dataHandle = osThreadCreate(osThread(printf_data), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)//这个是motor任务的代码
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	MOTOR_Init(&motor_6020);
  while(1)
  {
    motor_6020.speed=GetSpeed;//速度环
		motor_6020.Set_speed=(channels[1]-1500)*300/500;
		motor_6020.give_Voltage=(int16_t)PID_Speed_Calculate(&motor_6020.speed_PID,motor_6020.speed,motor_6020.Set_speed);
				
		
//		motor_6020.position=GetAngle;//位置环(能控但是有点慢且不准)
////		//进行简单的滤波
////		if(last_second_channels-channels[2]>1&&last_second_channels-channels[2]<-1)
////		{
////			motor_6020.Set_position=(channels[2]-1000)*720/1000;
////		}
//		motor_6020.Set_position=(channels[2]-1000)*720/1000;
//		motor_6020.give_Voltage=(int16_t)PID_Position_Calculate(&motor_6020.position_PID,motor_6020.position,motor_6020.Set_position);
		
//		if(Remote_Protected_Flag)//遥控保护
//		{
//			Remote_Protected_time++;
//			if(Remote_Protected_time>Remote_Protected_MAX_time)
//			{
//				motor_6020.give_Voltage=0;
//			}
//		}
//		
    CAN_TxHeaderTypeDef pHeader1;//控制电机电压
		pHeader1.StdId = 0x1FF;       
		pHeader1.IDE = CAN_ID_STD;    
		pHeader1.RTR = CAN_RTR_DATA;  
		pHeader1.DLC = 0x08;          

		uint8_t aData[8]={0};
		aData[0] = motor_6020.give_Voltage >> 8;          
		aData[1] = motor_6020.give_Voltage;
//		aData[0] = SetVoltage >> 8;          
//		aData[1] = SetVoltage;    		
		aData[2] = 0;          
		aData[3] = 0; 
		aData[4] = 0;          
		aData[5] = 0; 
		aData[6] = 0;          
		aData[7] = 0; 
		
		HAL_CAN_AddTxMessage(&hcan1, &pHeader1, aData, 0);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the printf_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)//优先级比电机驱动高，后面可能得改一下
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		HAL_UART_Receive_IT(&huart3,rx_buffer,32);
		osDelay(3);
//		last_second_channels++;
//		if(last_second_channels>300)
//		{
//			last_second_channels=channels[2];
//		}
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//回调函数
{
CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]={0};
    HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &rx_header, rx_data);//将数据存放于rx_data数组中
 
		switch (rx_header.StdId)//这个以及下面自己发挥
    {
 
        
				case 0x205://根据电机具体id号设置 0x204+id(6020手册上找)
        {
            Last_Get_Real_Angle=Get_Real_Angle;
						Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            GetTemperature=(uint16_t) (rx_data)[6] ;
						if(Last_Get_Real_Angle-Get_Real_Angle>200)
						{
								number++;
						}
						if(Last_Get_Real_Angle-Get_Real_Angle<-200)
						{
							number--;
						}
						GetAngle=Get_Real_Angle+number*360;
            break;																										// 此处是将两个数据合并为一个数据
        }
				
    }

	}

/* USER CODE END Application */
