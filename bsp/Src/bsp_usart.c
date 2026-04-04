#include "bsp_usart.h"
#include <stdint.h>

/* reference: https://zhuanlan.zhihu.com/p/720966722 */

/* receiption buffers */
__attribute__((section(".D1_SECTION"))) uint8_t uart5_rx_buff[2][UART5_RX_BUFF_LEN]; //dbus
__attribute__((section(".D1_SECTION"))) uint8_t uart7_rx_buff[2][UART7_RX_BUFF_LEN]; //vt03
__attribute__((section(".D1_SECTION"))) uint8_t uart10_rx_buff[2][UART10_RX_BUFF_LEN]; // referee

/* weak functions for data interpretation */
__weak void uart5_data_interpret(uint8_t *rx_buff, uint16_t received_len);
__weak void uart7_data_interpret(uint8_t *rx_buff, uint16_t received_len);
__weak void uart10_data_interpret(uint8_t *rx_buff, uint16_t received_len);

/* struct config for double dma setting */
typedef void (*uart_interpret_func)(uint8_t *rx_buff, uint16_t received_len);
struct uart_rx_config {
	uint8_t *buff1;
	uint8_t *buff2;
	uint16_t len_buff;
    uart_interpret_func interpret_func;    /* Data interpretation function for fixed length */
};

struct uart_rx_config uart5_config = {
	.buff1 = uart5_rx_buff[0],
	.buff2 = uart5_rx_buff[1],
	.len_buff = UART5_RX_BUFF_LEN,
	.interpret_func = uart5_data_interpret,
};

struct uart_rx_config uart7_config = {
	.buff1 = uart7_rx_buff[0],
	.buff2 = uart7_rx_buff[1],
	.len_buff = UART7_RX_BUFF_LEN,
	.interpret_func = uart7_data_interpret,
};

struct uart_rx_config uart10_config = {
	.buff1 = uart10_rx_buff[0],
	.buff2 = uart10_rx_buff[1],
	.len_buff = UART10_RX_BUFF_LEN,
	.interpret_func = uart10_data_interpret,
};

static HAL_StatusTypeDef dma_rx_double_buff_init(UART_HandleTypeDef *huart, 
	struct uart_rx_config *config)
{
		/* UART IDLE reception mode */
		huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
		huart->RxEventType = HAL_UART_RXEVENT_IDLE;
		huart->RxXferSize = config->len_buff;

		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); /* Enable DMA */
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);     /* Enable IDLE interrupt */

		/* Configure DMA double buffer */
		return HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
				   (uint32_t)config->buff1, (uint32_t)config->buff2, config->len_buff);
}

static void uart_rx_handler_double_dma(UART_HandleTypeDef *huart, uint16_t size, 
	struct uart_rx_config *config)
{
		
		__HAL_DMA_DISABLE(huart->hdmarx); /* Disable DMA */

		/* Check DMA current buffer */
		if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) ==
	    		RESET) {
				/* Change DMA buffer and reset NDTR */
				((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
		
				config->interpret_func(config->buff1, size);
		} else {
				((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
				config->interpret_func(config->buff2, size);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, config->len_buff); /* reset length */
		__HAL_DMA_ENABLE(huart->hdmarx); /* Enable DMA */
}

HAL_StatusTypeDef usart_init(void)
{
	uint8_t return_value = HAL_OK;
	if (HAL_OK != dma_rx_double_buff_init(&huart5, &uart5_config))
		return_value = HAL_ERROR;
	if (HAL_OK != dma_rx_double_buff_init(&huart7, &uart7_config))
		return_value = HAL_ERROR;
 	if (HAL_OK != dma_rx_double_buff_init(&huart10, &uart10_config))
		return_value = HAL_ERROR;

	return return_value;
}

/* rewrite HAL UART RX Event callback function */ 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart5) {
		uart_rx_handler_double_dma(huart, Size, &uart5_config);
	} else if (huart == &huart7) {
		uart_rx_handler_double_dma(huart, Size, &uart7_config);
	} else if (huart == &huart10) {
		uart_rx_handler_double_dma(huart, Size, &uart10_config);
	}
}
