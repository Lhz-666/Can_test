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
uint16_t GetSpeed,SetSpeed;
int16_t speed=5000;
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
  /* Infinite loop */
	for(;;)//这段CMD代码一定要放在循环中否则动不了。还有就是由于这里没有电调，所以直接用的是电机的标识符
  {
    CAN_TxHeaderTypeDef pHeader1;
		pHeader1.StdId = 0x1FF;       //��ʶ����0x205
		pHeader1.IDE = CAN_ID_STD;    //֡���ͣ���׼֡
		pHeader1.RTR = CAN_RTR_DATA;  //֡��ʽ��DATA
		pHeader1.DLC = 0x08;          //DLC��8�ֽ�
		
		uint8_t aData[8]={0};
//		uint8_t Data=20;
		aData[0] = speed >> 8;          //���ID��1�����Ƶ���ֵ��8λ
		aData[1] = speed;               //���ID��1�����Ƶ�����8λ
		aData[2] = 0;          //���ID��1�����Ƶ���ֵ��8λ
		aData[3] = 0; 
		aData[4] = 0;          //���ID��1�����Ƶ���ֵ��8λ
		aData[5] = 0; 
		aData[6] = 0;          //���ID��1�����Ƶ���ֵ��8λ
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
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
//    int a = 10, b = 20;
//		printf("a = %d, b = %d\n", a, b);  // 两个 %d，两个参数
//		uint8_t ch=0;
//	HAL_UART_Receive(&huart1,&ch,1,0xffff);
//		osDelay(1);
		int a = 10, b = 20;
		printf("a = %d, b = %d\n", a, b);  // 两个 %d，两个参数
		osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//回调函数
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//将数据存放于rx_data数组中
 
    switch (rx_header.StdId)//这个以及下面自己发挥
    {
 
        case 0x205://根据电机具体id号设置 0x204+id(6020手册上找)
        {
 
						GetSpeed = (uint16_t)((rx_data)[2] << 8 | (rx_data)[3]); // 根据手册 2、3 位分别为电机转速的高八位、低八位
            break;																										// 此处是将两个数据合并为一个数据
        }
 
    }
	}
/* USER CODE END Application */
