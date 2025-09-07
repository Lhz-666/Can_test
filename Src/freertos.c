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
#include "inv_mpu.h"
//void can_filter_init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/*pitch电机*/
int16_t pitch_GetSpeed=0,pitch_GetAngle=0,pitch_GetCurrent=0,pitch_GetTemperature=0;
int16_t pitch_Last_Get_Real_Angle=0,pitch_Get_Real_Angle=0;//获取的真实值
/*yaw电机*/
int16_t yaw_GetSpeed=0,yaw_GetAngle=0,yaw_GetCurrent=0,yaw_GetTemperature=0;
int16_t yaw_Last_Get_Real_Angle=0,yaw_Get_Real_Angle=0;
/*左摩擦轮*/
int16_t left_mocan_GetSpeed=0,left_mocan_GetAngle=0,left_mocan_GetCurrent=0,left_mocan_GetTemperature=0;
int16_t left_mocan_Last_Get_Real_Angle=0,left_mocan_Get_Real_Angle=0;
/*右摩擦轮*/
int16_t right_mocan_GetSpeed=0,right_mocan_GetAngle=0,right_mocan_GetCurrent=0,right_mocan_GetTemperature=0;
int16_t right_mocan_Last_Get_Real_Angle=0,right_mocan_Get_Real_Angle=0;
/*拨盘电机*/
int16_t bopan_GetSpeed=0,bopan_GetAngle=0,bopan_GetCurrent=0,bopan_GetTemperature=0;
int16_t bopan_Last_Get_Real_Angle=0,bopan_Get_Real_Angle=0;
/*mpu的数据*/
float pitch,yaw,roll;


int16_t SetVoltage=12500;//这里是电压值。电压给定值范围：-25000~0~25000, 对应最大转矩电流范围 -3A~0~3A
int8_t pitch_number1,yaw_number2;//这里是圈数

int16_t setspeed=0;
int16_t pitch_setposition=0;
int16_t yaw_setposition=0;

extern uint8_t rx_buffer[32];

extern uint16_t channels[32];

MOTOR_t pitch_motor_6020,yaw_motor_6020,left_mocan_2006,right_mocan_2006,bopan_3508;

//int pitch_motor_give_Voltage=0,yaw_give_Voltage=0,left_mocan_give_Voltage=0,right_mocan_give_Voltage=0,bopan_give_Voltage=0;

//uint16_t pitch
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	MOTOR_Init(&pitch_motor_6020);
	MOTOR_Init(&yaw_motor_6020);
	MOTOR_Init(&left_mocan_2006);
	MOTOR_Init(&right_mocan_2006);
	MOTOR_Init(&bopan_3508);
	channels[0]=1500;
	channels[1]=1500;
	channels[2]=1500;
	channels[3]=1500;
	channels[4]=1500;
  /* Infinite loop */
	for(;;)
  {
    if(channels[0]==1500&&channels[1]==1501&&channels[2]==1500&&channels[3]==1498&&channels[4]==1000&&channels[5]==1000)//遥控保护相关的，判断的值通过遥控器设定
		{
			pitch_motor_6020.give_Voltage=0;
			yaw_motor_6020.give_Voltage=0;
			left_mocan_2006.give_Voltage=0;
			right_mocan_2006.give_Voltage=0;
			bopan_3508.give_Voltage=0;
		}
		else
		{
			/*摩擦轮*/
			left_mocan_2006.Set_speed=(channels[1]-1500)*6000/500;
//			left_mocan_2006.Set_speed=setspeed;
			left_mocan_2006.speed=left_mocan_GetSpeed;//速度环
			left_mocan_2006.give_Voltage=(int16_t)PID_Speed_Calculate(&left_mocan_2006.speed_PID,left_mocan_2006.speed,left_mocan_2006.Set_speed);
				
		right_mocan_2006.Set_speed=(channels[1]-1500)*6000/500;
//			right_mocan_2006.Set_speed=setspeed;
			right_mocan_2006.speed=right_mocan_GetSpeed;//速度环
			right_mocan_2006.give_Voltage=(int16_t)PID_Speed_Calculate(&right_mocan_2006.speed_PID,right_mocan_2006.speed,right_mocan_2006.Set_speed);
    /*pitch和yaw轴*/
//		pitch_motor_6020.position=pitch_GetAngle;//位置环//高低
//		pitch_motor_6020.Set_position=(channels[2]-1000)*30/1000;
		pitch_motor_6020.position=pitch;
		pitch_motor_6020.Set_position=pitch_setposition;
		pitch_motor_6020.give_Voltage=(int16_t)PID_Position_Calculate(&pitch_motor_6020.position_PID,pitch_motor_6020.position,pitch_motor_6020.Set_position);
			
//		yaw_motor_6020.position=yaw_GetAngle;//位置环//左右
//		yaw_motor_6020.Set_position=(channels[0]-1000)*360/1000;
		yaw_motor_6020.position=yaw;//位置环//左右
		yaw_motor_6020.Set_position=yaw_setposition;
		yaw_motor_6020.give_Voltage=(int16_t)PID_Position_Calculate(&yaw_motor_6020.position_PID,yaw_motor_6020.position,yaw_motor_6020.Set_position);
		}
		/*控制pitch电机电压(CAN1)*/
    CAN_TxHeaderTypeDef pHeader1;
		pHeader1.StdId = 0x1FF;       
		pHeader1.IDE = CAN_ID_STD;    
		pHeader1.RTR = CAN_RTR_DATA;  
		pHeader1.DLC = 0x08;          

		uint8_t aData[8]={0};
		aData[0] = 0;          
		aData[1] = 0;		
		aData[2] = pitch_motor_6020.give_Voltage >> 8;          
		aData[3] = pitch_motor_6020.give_Voltage ; 
		aData[4] = 0;          
		aData[5] = 0; 
		aData[6] = 0;          
		aData[7] = 0; 
		
		HAL_CAN_AddTxMessage(&hcan1, &pHeader1, aData, 0);
		/*控制左右摩擦轮和拨盘电机电压(CAN1)*/
		CAN_TxHeaderTypeDef pHeader3;
		pHeader3.StdId = 0x200;       
		pHeader3.IDE = CAN_ID_STD;    
		pHeader3.RTR = CAN_RTR_DATA;  
		pHeader3.DLC = 0x08;          

		uint8_t cData[8]={0};
		cData[0] = left_mocan_2006.give_Voltage>>8;          
		cData[1] = left_mocan_2006.give_Voltage;		
		cData[2] = right_mocan_2006.give_Voltage >> 8;          
		cData[3] = right_mocan_2006.give_Voltage ; 
		cData[4] = bopan_3508.give_Voltage;          
		cData[5] = bopan_3508.give_Voltage; 
		cData[6] = 0;          
		cData[7] = 0; 
		
		HAL_CAN_AddTxMessage(&hcan1, &pHeader3, cData, 0);
		/*控制yaw电机电压(CAN2)*/
		CAN_TxHeaderTypeDef pHeader2;
		pHeader2.StdId = 0x1FF;       
		pHeader2.IDE = CAN_ID_STD;    
		pHeader2.RTR = CAN_RTR_DATA;  
		pHeader2.DLC = 0x08;          

		uint8_t bData[8]={0};
		bData[0] = 0;          
		bData[1] = 0;  		
		bData[2] = 0;          
		bData[3] = 0; 
		bData[4] = 0;          
		bData[5] = 0; 
		bData[6] = yaw_motor_6020.give_Voltage >> 8;          
		bData[7] = yaw_motor_6020.give_Voltage; 
		
		HAL_CAN_AddTxMessage(&hcan2, &pHeader2, bData, 0);

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
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
//		printf("Rpm:%d,Speed:%d,Current:%d,Temperature:%d\n", GetRpm,GetSpeed,GetCurrent,GetTemperature);  // 两个 %d，两个参数
		printf("%d,%d\n", (int)yaw,yaw_setposition);  //打印
		HAL_UART_Receive_IT(&huart3,rx_buffer,32);
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		osDelay(1);
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
 
        case 0x206://根据电机具体id号设置 0x204+id(6020手册上找)//pitch轴
        {
             pitch_Last_Get_Real_Angle=pitch_Get_Real_Angle;
						pitch_Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            pitch_GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            pitch_GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            pitch_GetTemperature=(uint16_t) (rx_data)[6] ;
					if(pitch_Last_Get_Real_Angle-pitch_Get_Real_Angle>200)
						{
								pitch_number1++;
						}
						if(pitch_Last_Get_Real_Angle-pitch_Get_Real_Angle<-200)
						{
							pitch_number1--;
						}
						pitch_GetAngle=pitch_Get_Real_Angle+pitch_number1*360;
            break;																										// 此处是将两个数据合并为一个数据
        }
				case 0x201://0x200+id左摩擦轮
				{
					left_mocan_Get_Real_Angle=left_mocan_Get_Real_Angle;
						left_mocan_Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            left_mocan_GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            left_mocan_GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            left_mocan_GetTemperature=(uint16_t) (rx_data)[6] ;
            break;																										// 此处是将两个数据合并为一个数据
				}
				case 0x202://右摩擦轮
				{
						right_mocan_Get_Real_Angle=right_mocan_Get_Real_Angle;
						right_mocan_Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            right_mocan_GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            right_mocan_GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            right_mocan_GetTemperature=(uint16_t) (rx_data)[6] ;
            break;																										// 此处是将两个数据合并为一个数据
				}
				case 0x203://拨盘
				{
						bopan_Get_Real_Angle=bopan_Get_Real_Angle;
						bopan_Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            bopan_GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            bopan_GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            bopan_GetTemperature=(uint16_t) (rx_data)[6] ;
            break;																										// 此处是将两个数据合并为一个数据
				}
 
    }
//	GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)//回调函数
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]={0};
    HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO1, &rx_header, rx_data);//将数据存放于rx_data数组中
 
    switch (rx_header.StdId)//这个以及下面自己发挥
    {
 
        case 0x208://根据电机具体id号设置 0x204+id(6020手册上找)
        {
             yaw_Last_Get_Real_Angle=yaw_Get_Real_Angle;
						yaw_Get_Real_Angle=((uint16_t)((rx_data)[0] << 8 | (rx_data)[1]))*360/8192;
            yaw_GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            yaw_GetCurrent=(uint16_t)((rx_data)[4] << 8 | (rx_data)[5]);
            yaw_GetTemperature=(uint16_t) (rx_data)[6] ;
					if(yaw_Last_Get_Real_Angle-yaw_Get_Real_Angle>200)
						{
								yaw_number2++;
						}
						if(yaw_Last_Get_Real_Angle-yaw_Get_Real_Angle<-200)
						{
							yaw_number2--;
						}
						yaw_GetAngle=yaw_Get_Real_Angle+yaw_number2*360;
            break;																										// 此处是将两个数据合并为一个数据
        }
 
    }
//	GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
	}
/* USER CODE END Application */
