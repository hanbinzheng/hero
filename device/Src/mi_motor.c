#include "mi_motor.h"
#include "bsp_dwt.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define MI_POS_MAX (-1.2981f)
#define MI_POS_MIN (-0.3179f)
#define MI_DIFF_MAX (PI / 6)
#define PITCH_TOTAL_SCALE (0.98023f) /* evaluation - depression */

int mi_enabled = 114514;
int mi_reception = 0;

struct mi_motor mi_motor = {
    .motor_id = 0x01,
    .error_bits = 0x00,
    .mode = MI_DISABLED,

    /* MIT control parameters */
    .kp = 40.0f,
    .kd = 2.0f,
};

static inline void limit(float *value, float min, float max)
{
	if (*value > max) {
		*value = max;
	} else if (*value < min) {
		*value = min;
	}
}

/* enable mi motor, hardcoded now */
void mi_motor_enable(void)
{
	uint8_t tx_data[8] = {0}; /* dummy data */
	uint32_t identifier =
	    (3 << 24) | (mi_motor.master_id << 8) | mi_motor.motor_id;
	do {
		mi_enabled = can_transmit(&hfdcan3, identifier, CAN_ID_EXT, tx_data);
		if (HAL_OK != mi_enabled)
			Error_Handler();
		dwt_delay_ms(1);
	} while (mi_motor.status != MI_ENABLED);
}

void mi_motor_disable(void)
{
	uint8_t tx_data[8] = {0};
	uint32_t identifier =
	    (4 << 24) | (mi_motor.master_id << 8) | mi_motor.motor_id;
	can_transmit(&hfdcan3, identifier, CAN_ID_EXT, tx_data);
	mi_motor.status = MI_DISABLED;
}

void mi_motor_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *rx_buff,
			struct mi_motor *mi)
{
	mi_reception++;

	uint32_t identifier = header->Identifier;
	uint8_t master_id = (uint8_t)(identifier & 0xFF);
	uint8_t motor_id = (uint8_t)((identifier >> 8) & 0xFF);
	uint8_t error_bits = (uint8_t)((identifier >> 16) & 0x3F);
	uint8_t motor_mode = (uint8_t)((identifier >> 22) & 0x03);
	uint8_t message_type = (uint8_t)((identifier >> 24) & 0x1F);

	/* reject all other message type and all other motor */
	if (message_type != 2 || motor_id != mi_motor.motor_id)
		return;

	mi_motor.master_id = master_id;
	mi_motor.error_bits = error_bits;
	mi_motor.mode = motor_mode;

	/* 0 ~ 65535 -> -4pi ~ 4pi */
	mi_motor.raw_pos = (int32_t)(rx_buff[1] | (rx_buff[0] << 8)) - 32767;
	mi_motor.pos = (float)(mi_motor.raw_pos) / (32768.0f / (4 * PI));
	/* 0 ~ 65535 -> -30rad/s ~ 30rad/s  */
	mi_motor.raw_vel = (int32_t)(rx_buff[3] | (rx_buff[2] << 8)) - 32767;
	mi_motor.vel = (float)(mi_motor.raw_vel) / (32768.0f / 30.0f);
	/* 0 ~ 65535 -> -12 Nm ~ 12 Nm */
	mi_motor.raw_trq = (int32_t)(rx_buff[5] | (rx_buff[4] << 8)) - 32767;
	mi_motor.trq = (float)(mi_motor.raw_trq) / (32768.0f / 12.0f);
	/* current temperature x 10 */
	mi_motor.raw_temp = (int32_t)(rx_buff[7] | (rx_buff[8] << 8));
	mi_motor.temp = (float)(mi_motor.raw_temp) / 10.0f;

	if (mi_motor.error_bits == 0 && mi_motor.mode == MOTOR_MODE) {
		mi_motor.status = MI_ENABLED;
	} else {
		mi_motor.status = MI_DISABLED;
	}
}

/* if kp < 0 or kd < 0, use the default value */
HAL_StatusTypeDef mi_send_command(float trq, float pos, float vel, float kp,
				  float kd)
{
	if (mi_motor.status != MI_ENABLED)
		return HAL_ERROR;

	/* if nagetive, use the default value */
	if (kp < 0)
		kp = mi_motor.kp;
	if (kd < 0)
		kd = mi_motor.kd;
	/* kp: 0 ~ 500.0 -> 0 ~ 65536, kd: 0 ~ 5.0 -> 0 ~ 65536 */
	limit(&kp, 0.0f, 500.0f);
	uint16_t kp_int = (uint16_t)(kp * (65536.0f / 500.0f));
	limit(&kd, 0.0f, 5.0f);
	uint16_t kd_int = (uint16_t)(kd * (65536.0f / 5.0f));

	limit(&trq, -12.0f, 12.0f);
	limit(&vel, -30.0f, 30.0f);
	limit(&pos, -4 * PI, 4 * PI);
	int32_t pos_int = (int32_t)(pos * (32768.0f / (4 * PI))) + 32767;
	int32_t vel_int = (int32_t)(vel * (32768.0f / 30.0f)) + 32767;
	uint16_t trq_int = (uint16_t)((int32_t)(trq * (32768.0f / 12.0f)) + 32767);

	uint8_t tx_data[8] = {0};
	tx_data[0] = (uint8_t)(pos_int >> 8);
	tx_data[1] = (uint8_t)(pos_int & 0xFF);
	tx_data[2] = (uint8_t)(vel_int >> 8);
	tx_data[3] = (uint8_t)(vel_int & 0xFF);
	tx_data[4] = (uint8_t)(kp_int >> 8);
	tx_data[5] = (uint8_t)(kp_int & 0xFF);
	tx_data[6] = (uint8_t)(kd_int >> 8);
	tx_data[7] = (uint8_t)(kd_int & 0xFF);

	uint32_t identifier = (1 << 24) | (trq_int << 8) | mi_motor.motor_id;
	return can_transmit(&hfdcan3, identifier, CAN_ID_EXT, tx_data);
}

HAL_StatusTypeDef mi_set_pos(float pos)
{
	float pos_measure = mi_motor.pos;
	if (pos - pos_measure >= MI_DIFF_MAX) {
		pos = pos_measure + MI_DIFF_MAX;
	} else if (pos - pos_measure <= - MI_DIFF_MAX) {
		pos = pos_measure - MI_DIFF_MAX;
	}

	return mi_send_command(0.0f, pos, 2, -1, -1); /* vel = 2, kp and kd: default value */
}