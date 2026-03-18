#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_

#include "fdcan.h"
#include <stdint.h>

/* CAN ID type, only Classical CAN is supported. */
enum can_id_type {
	CAN_ID_STD = 0,
	CAN_ID_EXT,
};

/* Bad design, to ba absorted, only for temporary usage */
HAL_StatusTypeDef can_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
			       enum can_id_type type, uint8_t *buff);

HAL_StatusTypeDef can_init(void);
#endif /* BSP_FDCAN_H_ */
