#ifndef BSP_SPI_H_
#define BSP_SPI_H_

#include "spi.h"
#include <stdint.h>

/* transfer a byte, and return the received data*/
uint8_t spi_tx_byte(SPI_HandleTypeDef *hspi, uint8_t tx_data);

#endif /* BSP_SPI_H_ */
