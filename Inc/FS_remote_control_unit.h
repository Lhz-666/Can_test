#ifndef FS_REMOTE_CONTROL_UNIT_H_
#define FS_REMOTE_CONTROL_UNIT_H_
#define IBUS_USER_CHANNELS 6//use 6channnels
/*功能：打开接收中断
*/
//void IBUS_INIT(void);

//void IBUS_Read_Channel(uint8_t user_channels);//用状态机的思想来接收数据
void parse_ibus_data(uint8_t *data) ;

extern UART_HandleTypeDef huart3;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) ;
#endif
