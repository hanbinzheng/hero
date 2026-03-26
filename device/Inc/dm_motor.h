#ifndef DM4310_MOTOR_H_
#define DM4310_MOTOR_H_

#include "main.h"
#include <stdint.h>

#ifndef PI
#define PI (3.14159265358979f)
#endif

#ifndef RPM_TO_RADS
#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)
#endif

#ifndef ANGLE_TO_RADS
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f)
#endif

#define DM4310_POS_MAX (200.0f)
#define DM4310_VEL_MAX (30.0f)
#define DM4310_TRQ_MAX (10.0f)

enum dm_motor_type {
	J4310 = 0,
	J6006,
};

struct dm_motor {
	uint8_t state;
	uint16_t raw_vel;
	uint16_t raw_cur;
	uint16_t raw_pos;
	uint16_t raw_trq;
	uint8_t temp_motor;
	uint8_t temp_mos;

	float pos;
	float vel;
	float trq;
	float cur;

	enum dm_motor_type type;
};

void dm_motor_interpret(uint8_t *rx_buff, uint8_t is_1t4, struct dm_motor *motor);
HAL_StatusTypeDef dm4310_enable(void);
HAL_StatusTypeDef dm4310_disable(void);
HAL_StatusTypeDef dm4310_send_command(float pos, float vel);
HAL_StatusTypeDef dm6006_set_vel(float vel);
float dm6006_get_pos(int pos);
float dm6006_set_pos(float pos);

extern struct dm_motor dm4310;
extern struct dm_motor dm6006;

#endif /* DM4310_MOTOR_H_ */
