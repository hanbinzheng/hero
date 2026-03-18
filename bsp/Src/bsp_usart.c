#include "bsp_usart.h"

#include <stdint.h>

/* receive buffers */
__attribute__((section(".dma12_buffer"))) uint8_t usart5_rx_buff[2][USART5_RX_FRAME_LEN];
__attribute__((section(".dma12_buffer"))) uint8_t usart7_rx_buff[2][USART7_RX_FRAME_LEN];

/* weak functions for data interpretation */
__weak void usart5_data_interpret(uint8_t *rx_buff);
__weak void usart7_data_interpret(uint8_t *rx_buff);

static void dma_rx_double_buff_init(UART_HandleTypeDef *huart, uint32_t *first_buff,
				    uint32_t *second_buff, uint32_t len_data)
{
	/* UART IDLE reception mode */
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	huart->RxEventType = HAL_UART_RXEVENT_IDLE;

	huart->RxXferSize = len_data; /* len_data = 2 x data */

	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); /* Enable DMA */
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);     /* Enable IDLE interrupt */

	/* Configure DMA double buffer */
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&huart->Instance->RDR,
				   (uint32_t)first_buff, (uint32_t)second_buff, len_data);
}

static void uart5_rx_handler(UART_HandleTypeDef *huart, uint16_t size)
{
	__HAL_DMA_DISABLE(huart->hdmarx); /* Disable DMA */

	/* Check DMA current buffer */
	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) ==
	    RESET) {
		/* Change DMA buffer and reset NDTR */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* stm32h7xx_hal_uart.c, line 2392 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART5_RX_BUFF_LEN);

		/* Interpret data if full frame is received */
		if (size == USART5_RX_FRAME_LEN)
			usart5_data_interpret(usart5_rx_buff[0]);
	} else {
		/* Change buffer and reset NDTR */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART5_RX_BUFF_LEN);

		if (size == USART5_RX_FRAME_LEN)
			usart5_data_interpret(usart5_rx_buff[1]);
	}

	__HAL_DMA_ENABLE(huart->hdmarx); /* Enable DMA */
}

static void uart7_rx_handler(UART_HandleTypeDef *huart, uint16_t size)
{
	__HAL_DMA_DISABLE(huart->hdmarx); /* Disable DMA */

	/* Check DMA current buffer */
	if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) ==
	    RESET) {
		/* Change DMA buffer and reset NDTR */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;

		/* stm32h7xx_hal_uart.c, line 2392 */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART7_RX_BUFF_LEN);

		/* Interpret data if full frame is received */
		if (size == USART7_RX_FRAME_LEN)
			usart7_data_interpret(usart7_rx_buff[0]);
	} else {
		/* Change buffer and reset NDTR */
		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART7_RX_BUFF_LEN);

		if (size == USART7_RX_FRAME_LEN)
			usart7_data_interpret(usart7_rx_buff[1]);
	}

	__HAL_DMA_ENABLE(huart->hdmarx); /* Enable DMA */
}

// reference:
// https://zhuanlan.zhihu.com/p/720966722

void usart_init(void)
{
	dma_rx_double_buff_init(&huart5, (uint32_t *)(usart5_rx_buff[0]),
				(uint32_t *)(usart5_rx_buff[1]), USART5_RX_BUFF_LEN);

	dma_rx_double_buff_init(&huart7, (uint32_t *)(usart7_rx_buff[0]),
				(uint32_t *)(usart7_rx_buff[1]), USART7_RX_BUFF_LEN);
}

// rewrite HAL UART RX Event callback function
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart5) {
		uart5_rx_handler(huart, Size);
	} else if (huart == &huart7) {
		uart7_rx_handler(huart, Size);
	}
}
