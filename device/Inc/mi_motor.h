#ifndef MI_H_
#define MI_H_

#include "bsp_fdcan.h"
#include <stdint.h>

enum mi_status_type {
	MI_DISABLED = 0x00,
	MI_ENABLED = 0x01,
	MI_ERROR = 0x02,
};

enum mi_motor_mode {
	/* this naming follows the mi manual, bad naming */
	RESET_MODE = 0,
	CALI_MODE = 1,	/* demarcate */
	MOTOR_MODE = 2, /* run time */
};

/* enum for error bits */
/* enum mi_error_info { */
/* }; */

struct mi_motor {
	uint8_t master_id;
	uint8_t motor_id;
	uint8_t error_bits; /* TODO */
	enum mi_motor_mode mode;
	enum mi_status_type status;

	float kp;
	float kd;

	int32_t raw_pos;
	int32_t raw_vel;
	int32_t raw_trq;
	int32_t raw_temp;
	float temp;
	float trq;
	float vel;
	float pos;
};

void mi_motor_enable(void);
void mi_motor_disable(void);
void mi_motor_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *rx_buff,
			struct mi_motor *mi);
HAL_StatusTypeDef mi_send_command(float trq, float pos, float vel, float kp,
				  float kd);
extern struct mi_motor mi_motor;

#endif /* MI_H_ */
