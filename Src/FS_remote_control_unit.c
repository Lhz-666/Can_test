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
uint16_t channels[IBUS_MAX_CHANNELS]={0};
uint16_t checksum_calculation,checksum_ibus;


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
				printf("deal with data\n");//这行代码删去后就不能正常控制了！！！
			HAL_UART_Receive_IT(&huart3, rx_buffer, IBUS_BUFFER_SIZE);
    }
}
