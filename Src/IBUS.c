#include "IBUS.h"


uint8_t  rx_buffer[32] = {0};
uint16_t channel[IBUS_USER_CHANNELS] = {0};
uint16_t checksum_cal, checksum_ibus;
uint8_t IBUS_RX_Finish = 0;



void IBUS_Init()
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;//���ýṹ��
	USART_InitTypeDef USART_InitStructure;//���ýṹ��
	NVIC_InitTypeDef NVIC_InitStructure;//���ýṹ��
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	
	//UART3_TX   GPIOB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//��������Ϊ50MHz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
  //UART3_RX	  GPIOB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ�� 
	
  //NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//USART1�����жϵ�ַ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  //USART ��ʼ������
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//ʹ��USART1ʱ��
	USART_InitStructure.USART_BaudRate = 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  //��ʼ������
  USART_Init(USART3, &USART_InitStructure); 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���1 
}


void IBUS_Handle()
{
	char txt[16];
	int i = 0,j = 0;
	uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};
	
	if(IBUS_RX_Finish==1)
	{
		IBUS_RX_Finish=0;//
		
		NVIC_DisableIRQ(USART3_IRQn);//�ر��ж�
		
			if(rx_buffer[0] == IBUS_start1 && rx_buffer[1] == IBUS_start2)
			{
				checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

				for( i = 0; i < IBUS_MAX_CHANNLES; i++)
				{
					channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
					checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
				}

				checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

				if(checksum_cal == checksum_ibus)   //������ֵУ�ԣ�У�Գɹ���װ��
				{
					for(j = 0; j < IBUS_USER_CHANNELS; j++)
					{
						channel[j] = channel_buffer[j];
					}
				}
			}
			
		NVIC_EnableIRQ(USART3_IRQn);//���ж�
		
	}

}



///****�жϺ���*****//
void USART3_IRQHandler(void)                	//�����жϷ������
	{
		unsigned char data;
		static uint8_t rxstart = 0; //״̬���ı�־λ
		static uint8_t rx_arr_flag = 0; //�������ݵ������±�
		uint8_t rx_getflag = 0;
		
		if(USART_GetITStatus(USART3, USART_IT_RXNE)  != RESET) 
		{
			/* �û����� */
			data = USART_ReceiveData(USART3);	//��ȡ���յ�������; 
			
				if (rxstart == 0 ) //״̬��
				{
					if(data == 0x20 && rx_getflag == 0) //�ж����ݵ���ʼλ��ֻ�Ǳ���ʹ�ã�������2�㱣��
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

	