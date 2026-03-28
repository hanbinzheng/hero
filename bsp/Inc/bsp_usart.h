#ifndef BSP_USART_H_
#define BSP_USART_H_

#include "usart.h"

#define UART5_RX_BUFF_LEN 256
#define UART7_RX_BUFF_LEN 256
#define UART10_RX_BUFF_LEN 256

HAL_StatusTypeDef usart_init(void);

#endif /* BSP_USART_H_ */
