#ifndef __IBUS_H
#define __IBUS_H

#include "usart.h"

#define IBUS_start1			0x20	    // 32 bytes
#define IBUS_start2			0x40	    // Command to set servo or motor speed is always 0x40
#define IBUS_USER_CHANNELS		10			// Use 6 channels
#define IBUS_MAX_CHANNLES		14

extern uint8_t IBUS_RX_Finish; 
extern uint8_t rx_buffer[32];
extern uint16_t channel[IBUS_USER_CHANNELS];

void IBUS_Handle(void);
void IBUS_Init(void);


#endif /* __IBUS_H */
