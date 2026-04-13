#include "bsp_spi.h"

uint8_t spi_tx_byte(SPI_HandleTypeDef *hspi, uint8_t tx_data)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(hspi, &tx_data, &rx_data, 1, 100);
	return rx_data;
}
