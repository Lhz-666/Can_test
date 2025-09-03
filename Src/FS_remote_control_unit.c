#include "main.h"
#include "usart.h"

#define IBUS_UART (&huart3)//使用的串口
#define IBUS_UART_INSTANCE (USART3)
#define IBUS_USER_CHANNELS 4//use 6channnels

#define IBUS_FIST_BYTE 0x20
#define IBUS_SECOND_BYTE 0x40
#define IBUS_MAX_CHANNELS 14
#define IBUS_BUFFER_SIZE 32

uint8_t rx_buffer[IBUS_BUFFER_SIZE]={0};
//uint16_t channel[IBUS_USER_CHANNELS]={0};
uint16_t channels[IBUS_MAX_CHANNELS]={0};
uint16_t checksum_calculation,checksum_ibus;
///*功能：打开接收中断
//*/
//void IBUS_Receive()
//{
//    HAL_UART_Receive_IT(IBUS_UART,rx_buffer,32);
//}

////void IBUS_Read_Channel(uint8_t user_channels)//用状态机的思想来处理接收到的数据
//void IBUS_Read_Channel()//用状态机的思想来处理接收到的数据
//{
//    uint16_t channel_buffer[IBUS_MAX_CHANNELS]={0};

//    if (rx_buffer[0]==IBUS_FIST_BYTE&&rx_buffer[1]==IBUS_SECOND_BYTE)//判断帧头
//    {
//        checksum_calculation=0xffff-rx_buffer[0]-rx_buffer[1];//用来检验
//        for(int i=0;i<IBUS_MAX_CHANNELS;i++)
//        {
//            channel_buffer[i]=(uint16_t)(rx_buffer[i*2+3])<<8|rx_buffer[i*2+2];
//            checksum_calculation = checksum_calculation - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
//        }

//        checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];//用来校验
//				if(checksum_calculation == checksum_ibus)
//				{
////						for(int j = 0; j < user_channels; j++)
////						{
////								channel[j] = channel_buffer[j];
////						}
//					for(int j = 0; j < IBUS_USER_CHANNELS; j++)
//						{
//								channel[j] = channel_buffer[j];
//						}
//				}
//    }
//}

////void HAL_UART_ReceiveIdle(UART_HandleTypeDef *huart)
////{
//////    //当触发了串口空闲中断
//////    if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))//自行处理数据(这里用是遥控器控制速度和位置)
//////    {
////////        SetSpeed=(channel[1]-1500)*350;
////////				SetPosition=(channel[4]-1500)*8192;
////////				Remote_Protected_time=0;
////////				Remote_Protected_Flag=0;
//////			printf("%d,%d,%d,%d\n", channel[1],channel[2],channel[3],channel[4]); 
//////    }
////	
////}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//		IBUS_Read_Channel();
//}

void parse_ibus_data(uint8_t *data) {
    // 检查数据包起始字节
    if (data[0] != 0x20 || data[1] != 0x40) {
        return; // 数据包格式错误
    }

    // 提取通道数据
    for (int i = 0; i < 14; i++) {
        channels[i] = (data[2 + i * 2 + 1] << 8) | data[2 + i * 2];
    }
}

extern UART_HandleTypeDef huart3;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) 
			{
        parse_ibus_data(rx_buffer);
			printf("deal with data\n");
			HAL_UART_Receive_IT(&huart3, rx_buffer, IBUS_BUFFER_SIZE);
    }
}
