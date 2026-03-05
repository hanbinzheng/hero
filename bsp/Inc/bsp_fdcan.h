#ifndef _BSP_FDCAN_H_
#define _BSP_FDCAN_H_

#include "fdcan.h"
#include <stdint.h>

/* CAN ID type, only Classical CAN is supported. */
enum can_id_type {
	CAN_ID_STD = 0,
	CAN_ID_EXT,
};

struct can_filter_config {
	uint32_t rx_id;
	uint32_t rx_mask;
	enum can_id_type id_type;
	FDCAN_HandleTypeDef *hfdcan;
};

/* TODO: Make the design to be OOP */

typedef void (*can_rx_callback_t)(FDCAN_RxHeaderTypeDef *header, uint8_t *buff);

/* Must be called prior to can_init(), and should be called only once. */
HAL_StatusTypeDef can_config_filter(struct can_filter_config const *conf, uint16_t num);

HAL_StatusTypeDef can_init(void);

/* Register a single global callback for all CAN reception events. */
void can_register_rx_callback(can_rx_callback_t callback);

/* Bad design to ba absorted, only for temporary usage */
HAL_StatusTypeDef can_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
							   enum can_id_type type, uint8_t *buff);

#endif /* _BSP_FDCAN_H_ */
