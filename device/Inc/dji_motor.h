#ifndef MOTOR_H__
#define MOTOR_H__

#include "main.h"
#include <stdint.h>

#define GM6020_ANGLE_OFFSET_1 4041
#define GM6020_ANGLE_OFFSET_2 50
#define GM6020_ANGLE_OFFSET_3 657
#define GM6020_ANGLE_OFFSET_4 4090

#ifndef PI
#define PI (3.14159265358979f)
#endif
#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f)
// GM6020
#define GM6020_CURRENT_FLOAT_TO_INT(value)                                               \
	((int16_t)((value) * 16384.0f / 3.0f)) // -3A~0~3A, -16384~0~16384
#define GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 3.0f / 16384.0f)
#define GM6020_VOLTAGE_FLOAT_TO_INT(value)                                               \
	((int16_t)((value) * 25000.0f / 24.0f)) // -24v~0~24v, -25000~0~25000
// #define GM6020_VOLTAGE_FLOAT_TO_INT(value) (int16_t)((value)) // -25000~0~25000

// M3508
#define M3508_CURRENT_FLOAT_TO_INT(value)                                                \
	((int16_t)((value) * 16384.0f / 20.0f)) // -20A~0~20A, -16384~0~16384
#define M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 20.0f / 16384.0f)
#define M3508_REDUC_RATE (3591.0f / 187.0f)

// M2006
#define M2006_CURRENT_FLOAT_TO_INT(value)                                                \
	((int16_t)((value) * 10000.0f / 10.0f)) // -10A~0~10A, -10000~0~10000
#define M2006_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 10.0f / 10000.0f)

enum motor_type {
	M3508,
	M2006,
	GM6020,
};

struct motor_info {
	int16_t raw_vel; // rpm
	int16_t raw_cur;
	uint16_t raw_pos; // 0~8191: 0° ~ 360°
	int8_t temperature;

	float pos; // rad
	float vel; // rad/s
	float cur; // A

	enum motor_type type;
};

void dji_motor_interpret(uint8_t *rx_buff, struct motor_info *motor);
HAL_StatusTypeDef dji3508_set_chassis_vel(float vel[4]);
HAL_StatusTypeDef dji3508_set_armor_vel(float vel[6]);
HAL_StatusTypeDef dji6020_set_vel(float vel[4]);

float dji_get_pos(struct motor_info *motor, int offset);
HAL_StatusTypeDef dji6020_set_pos(float pos[4]);

extern struct motor_info dji3508_1;
extern struct motor_info dji3508_2;
extern struct motor_info dji3508_3;
extern struct motor_info dji3508_4;
extern struct motor_info dji3508_5;
extern struct motor_info dji3508_6;
extern struct motor_info dji3508_7;
extern struct motor_info dji3508_8;
extern struct motor_info dji3508_9;
extern struct motor_info dji3508_10;
extern struct motor_info dji6020_1;
extern struct motor_info dji6020_2;
extern struct motor_info dji6020_3;
extern struct motor_info dji6020_4;

#endif // MOTOR_H__
