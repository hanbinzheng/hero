#ifndef BSP_USART_H_
#define BSP_USART_H_

#include "usart.h"

#define USART5_RX_FRAME_LEN 18
#define USART5_RX_BUFF_LEN 36 /* 18 x 2 */
#define USART7_RX_FRAME_LEN 21
#define USART7_RX_BUFF_LEN 42

void usart_init(void);

#endif /* BSP_USART_H_ */
