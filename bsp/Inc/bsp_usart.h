#ifndef BSP_USART_H_
#define BSP_USART_H_

#include "usart.h"

// #define USART5_RX_FRAME_LEN 18
#define UART5_RX_BUFF_LEN 256
// #define USART7_RX_FRAME_LEN 21
#define UART7_RX_BUFF_LEN 256
#define UART10_RX_BUFF_LEN 256

void usart_init(void);

#endif /* BSP_USART_H_ */
