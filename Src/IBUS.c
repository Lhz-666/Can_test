#include "IBUS.h"


uint8_t  rx_buffer[32] = {0};
uint16_t channel[IBUS_USER_CHANNELS] = {0};
uint16_t checksum_cal, checksum_ibus;
uint8_t IBUS_RX_Finish = 0;



void IBUS_Init()
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;//调用结构体
	USART_InitTypeDef USART_InitStructure;//调用结构体
	NVIC_InitTypeDef NVIC_InitStructure;//调用结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3时钟
	
	//UART3_TX   GPIOB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设置速率为50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
  //UART3_RX	  GPIOB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化 
	
  //NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//USART1串口中断地址
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  //USART 初始化设置
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//使能USART1时钟
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  //初始化串口
  USART_Init(USART3, &USART_InitStructure); 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口1 
}


void IBUS_Handle()
{
	char txt[16];
	int i = 0,j = 0;
	uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};
	
	if(IBUS_RX_Finish==1)
	{
		IBUS_RX_Finish=0;//
		
		NVIC_DisableIRQ(USART3_IRQn);//关闭中断
		
			if(rx_buffer[0] == IBUS_start1 && rx_buffer[1] == IBUS_start2)
			{
				checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

				for( i = 0; i < IBUS_MAX_CHANNLES; i++)
				{
					channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
					checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
				}

				checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

				if(checksum_cal == checksum_ibus)   //进行数值校对，校对成功后装入
				{
					for(j = 0; j < IBUS_USER_CHANNELS; j++)
					{
						channel[j] = channel_buffer[j];
					}
				}
			}
			
		NVIC_EnableIRQ(USART3_IRQn);//打开中断
		
	}

}



///****中断函数*****//
void USART3_IRQHandler(void)                	//串口中断服务程序
	{
		unsigned char data;
		static uint8_t rxstart = 0; //状态机的标志位
		static uint8_t rx_arr_flag = 0; //保存数据的数组下标
		uint8_t rx_getflag = 0;
		
		if(USART_GetITStatus(USART3, USART_IT_RXNE)  != RESET) 
		{
			/* 用户代码 */
			data = USART_ReceiveData(USART3);	//读取接收到的数据; 
			
				if (rxstart == 0 ) //状态机
				{
					if(data == 0x20 && rx_getflag == 0) //判断数据的起始位，只是保险使用，还会有2层保险
					{
						rx_arr_flag = 0;
						rxstart = 1;
						rx_buffer[rx_arr_flag] = data;
						rx_arr_flag++;
					}
				}
				else if (rxstart == 1)
				{
					rx_buffer[rx_arr_flag] = data;
					rx_arr_flag++;
					if(rx_arr_flag >= 32)
					{
						rxstart = 2;
					}
				}
				else if (rxstart == 2)
				{
					if(rxstart == 2)
					{
						rxstart = 0;
						IBUS_RX_Finish = 1;
						rx_getflag = 1;
					}
				}
				
				USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		}
		
	}

	