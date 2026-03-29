#include "dm_motor.h"
#include "bsp_fdcan.h"
#include "pid.h"
#include <stdint.h>

struct dm_motor dm4310; /* master id: 0x0B, can id: 0x01 */
struct dm_motor dm6006;

#define DM6006_ZERO (4200) /* 4200 or 5600 */
#define DM6006_DIFF_MAX (0.50f)
#define DM6006_VEL_SCALE (0x1800)

/*
static inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
*/

/* map [0, 2^bit - 1] unsigned int to [x_min, x_max] float */
static inline float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 1 tuo 4 mode data interpretation */
static void feedback_interpret_1t4(uint8_t *rx_buff, struct dm_motor *motor)
{
	motor->raw_pos = (uint16_t)((rx_buff[0] << 8) | rx_buff[1]);
	motor->raw_vel = (uint16_t)((rx_buff[2] << 8) | rx_buff[3]);
	motor->raw_cur = (uint16_t)((rx_buff[4] << 8) | rx_buff[5]); /* mA */
	motor->temp_motor = rx_buff[6];
	motor->state = rx_buff[7];

	motor->pos = ANGLE_TO_RADS(motor->raw_pos);
	motor->vel =
	    RPM_TO_RADS((int16_t)motor->raw_vel) / 100.0f; /* 100 vel, in rpm */
	motor->cur = (float)((int16_t)motor->raw_cur) / 1000.0f; /* A */
}

/* general damiao motor data interpretation */
static void feedback_interpret(uint8_t *rx_buff, struct dm_motor *motor)
{
	motor->state = (uint8_t)rx_buff[0] >> 4;
	motor->raw_pos = (uint16_t)(rx_buff[1] << 8 | rx_buff[2]);
	motor->raw_vel = (uint16_t)(rx_buff[3] << 4 | rx_buff[4] >> 4);
	motor->raw_trq = (uint16_t)((rx_buff[4] & 0x0F) << 8 | rx_buff[5]);
	motor->temp_mos = rx_buff[6];
	motor->temp_motor = rx_buff[7];

	motor->pos =
	    uint_to_float(motor->raw_pos, -DM4310_POS_MAX, DM4310_POS_MAX, 16);
	motor->vel =
	    uint_to_float(motor->raw_vel, -DM4310_VEL_MAX, DM4310_VEL_MAX, 12);
	motor->trq =
	    uint_to_float(motor->raw_trq, -DM4310_TRQ_MAX, DM4310_TRQ_MAX, 12);
}

int dm_debug = 114514;
void dm_motor_interpret(uint8_t *rx_buff, uint8_t is_1t4, struct dm_motor *motor)
{
	/* 1 tuo 4: dji mode */
	if (is_1t4) {
		dm_debug = 1;
		feedback_interpret_1t4(rx_buff, motor);
	} else {
		dm_debug = 2;
		feedback_interpret(rx_buff, motor);
	}
}

HAL_StatusTypeDef dm4310_enable(void)
{
	uint8_t tx_buff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	return can_transmit(&hfdcan1, 0x101, CAN_ID_STD, tx_buff);
}

HAL_StatusTypeDef dm4310_disable(void)
{
	uint8_t tx_buff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	return can_transmit(&hfdcan1, 0x101, CAN_ID_STD, tx_buff);
}

HAL_StatusTypeDef dm4310_send_command(float pos, float vel)
{
	uint8_t tx_buff[8] = {0};
	*(float *)(&tx_buff[0]) = pos;
	*(float *)(&tx_buff[4]) = vel;

	return can_transmit(&hfdcan1, 0x101, CAN_ID_STD, tx_buff);
}

/* control for dm6006 */
static struct pid_info pid_6006_v2c = {
    .kp = 0.15f, .ki = 0.001f, .kd = 0.0f, .i_limit = 0.2f, .out_limit = 1.0f};
static struct pid_info pid_6006_p2v = {
    .kp = 10.0f, .ki = 0.0f, .kd = 800.0f, .i_limit = 0.0f, .out_limit = 15.0f};

static int16_t dm6006_float_to_int(float value, uint16_t scale)
{
	/* value should be limited to -1 and 1 */
	if (value > 1.0f) {
		value = 1.0f;
	} else if (value < -1.0f) {
		value = -1.0f;
	}

	value = value * scale;
	return (int16_t)value;
}

HAL_StatusTypeDef dm6006_set_vel(float ref_vel, float meas_vel) /* can id: 0x3FE */
{
	/* cur: -1.0 ~ 1.0 */
	static uint8_t low, high; /* for debug */
	float cur = pid_calculate(&pid_6006_v2c, ref_vel, meas_vel);
	int16_t cur_int = dm6006_float_to_int(cur, DM6006_VEL_SCALE);
	uint8_t tx_buff[8] = {0};
	low = (uint8_t)(cur_int & 0x00FF); /* low 8 bits */
	high = (uint8_t)(cur_int >> 8);	   /* high 8 bits */
	tx_buff[0] = low;
	tx_buff[1] = high;
	return can_transmit(&hfdcan1, 0x3FE, CAN_ID_STD, tx_buff);
}

float dm6006_get_pos(void)
{
	int pos = dm6006.raw_pos;
	pos = DM6006_ZERO - pos;
	while (pos >= 4096) {
		pos -= 8192;
	}
	while (pos <= -4096) {
		pos += 8192;
	}
	return pos / 4096.0f * 3.14159265358979f;
}

static float update_pos_ref(float ref, float meas)
{
	while (ref - meas >= PI) {
		ref -= 2 * PI;
	}
	while (ref - meas <= -PI) {
		ref += 2 * PI;
	}
	return ref;
}

HAL_StatusTypeDef dm6006_set_pos(float ref_pos, float meas_pos, float meas_vel)
{
	ref_pos = update_pos_ref(ref_pos, meas_pos);

	// if (pos - pos_measure >= DM6006_DIFF_MAX) {
	// 	pos = pos_measure + DM6006_DIFF_MAX;
	// } else if (pos - pos_measure <= -DM6006_DIFF_MAX) {
	// 	pos = pos_measure - DM6006_DIFF_MAX;
	// }

	float ref_vel = pid_calculate(&pid_6006_p2v, ref_pos, meas_pos);
	return dm6006_set_vel(ref_vel, meas_vel);
}