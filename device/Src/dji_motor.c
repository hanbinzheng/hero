#include "dji_motor.h"
#include "bsp_fdcan.h"
#include "pid.h"
#include "math.h"

int debug_motor = 0;

/*
 **************************************************************************
 * Motor Informations
 **************************************************************************
 */
/* chassis motors */
struct motor_info dji3508_1 = {.type = M3508};
struct motor_info dji3508_2 = {.type = M3508};
struct motor_info dji3508_3 = {.type = M3508};
struct motor_info dji3508_4 = {.type = M3508};
struct motor_info dji6020_1 = {.type = GM6020};
struct motor_info dji6020_2 = {.type = GM6020};
struct motor_info dji6020_3 = {.type = GM6020};
struct motor_info dji6020_4 = {.type = GM6020};

/* armor friction motors */
struct motor_info dji3508_5 = {.type = M3508};
struct motor_info dji3508_6 = {.type = M3508};
struct motor_info dji3508_7 = {.type = M3508};
struct motor_info dji3508_8 = {.type = M3508};
struct motor_info dji3508_9 = {.type = M3508};
struct motor_info dji3508_10 = {.type = M3508};

/* chassis 3508 motor pid */
static struct pid_info pid_3508v2c_1 = {
    .kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_2 = {
    .kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_3 = {
    .kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_4 = {
    .kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};

/* chassis 6020 motor pid*/
static struct pid_info pid_6020v2v_1 = {
    .kp = 1.2f, .ki = 0.1f, .kd = 0, .i_limit = 1.5f, .out_limit = 20};
static struct pid_info pid_6020v2v_2 = {
    .kp = 1.0f, .ki = 0.1f, .kd = 0, .i_limit = 1.5f, .out_limit = 20};
static struct pid_info pid_6020v2v_3 = {
    .kp = 1.0f, .ki = 0.12f, .kd = 0, .i_limit = 1.2f, .out_limit = 20};
static struct pid_info pid_6020v2v_4 = {
    .kp = 1.2f, .ki = 0.1f, .kd = 0, .i_limit = 1.5f, .out_limit = 20};

// static volatile struct pid_info pid_6020p2v_1 = {
//     .kp = 7.5f, .ki = 0.0f, .kd = 0, .i_limit = 0.0f, .out_limit = 15};
// static volatile struct pid_info pid_6020p2v_2 = {
//     .kp = 7.5f, .ki = 0.0f, .kd = 0, .i_limit = 0.0f, .out_limit = 15};
// static volatile struct pid_info pid_6020p2v_3 = {
//     .kp = 7.5f, .ki = 0.0f, .kd = 0, .i_limit = 0.0f, .out_limit = 15};
// static volatile struct pid_info pid_6020p2v_4 = {
//     .kp = 7.5f, .ki = 0.0f, .kd = 0, .i_limit = 0.0f, .out_limit = 15};

/* armor friction 3508 pid */
static struct pid_info pid_3508v2c_5 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};
static struct pid_info pid_3508v2c_6 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};
static struct pid_info pid_3508v2c_7 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};
static struct pid_info pid_3508v2c_8 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};
static struct pid_info pid_3508v2c_9 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};
static struct pid_info pid_3508v2c_10 = {
    .kp = 0.14f, .ki = 0.0005f, .kd = 0, .i_limit = 2.0, .out_limit = 20};


float dji_get_pos(struct motor_info *motor, int offset)
{
	/* - pi ~ pi, where offset is the zero part*/
	int angle = motor->raw_pos - offset;
	if (angle >= 4096) {
		angle -= 8192;
	} else if (angle < -4096) {
		angle += 8192;
	}

	return ANGLE_TO_RADS(angle);
}

static float update_pos_ref(float ref, float measure)
{
	while (ref - measure >= PI) {
		ref -= 2 * PI;
	}

	while (ref - measure <= -PI) {
		ref += 2 * PI;
	}

	return ref;
}

void dji_motor_interpret(uint8_t *rx_buff, struct motor_info *motor)
{
	// interpret feedback raw data
	motor->raw_pos = (rx_buff[0] << 8) | rx_buff[1];
	motor->raw_vel = (rx_buff[2] << 8) | rx_buff[3];
	motor->raw_cur = (rx_buff[4] << 8) | rx_buff[5];
	motor->temperature = rx_buff[6];

	// convert to physical values
	motor->pos = ANGLE_TO_RADS(motor->raw_pos);
	motor->vel = RPM_TO_RADS(motor->raw_vel);
	switch (motor->type) {
	case M3508:
		motor->cur = M3508_CURRENT_INT_TO_FLOAT(motor->raw_cur);
		break;
	case M2006:
		motor->cur = M2006_CURRENT_INT_TO_FLOAT(motor->raw_cur);
		break;
	case GM6020:
		motor->cur = GM6020_CURRENT_INT_TO_FLOAT(motor->raw_cur);
		break;
	default:
		break;
	}
}

HAL_StatusTypeDef dji3508_set_chassis_vel(float vel[4])
{
	/* 0x200 */
	float c1 = pid_calculate(&pid_3508v2c_1, vel[0] * M3508_REDUC_RATE, dji3508_1.vel);
	float c2 = pid_calculate(&pid_3508v2c_2, vel[1] * M3508_REDUC_RATE, dji3508_2.vel);
	float c3 = pid_calculate(&pid_3508v2c_3, vel[2] * M3508_REDUC_RATE, dji3508_3.vel);
	float c4 = pid_calculate(&pid_3508v2c_4, vel[3] * M3508_REDUC_RATE, dji3508_4.vel);

	uint16_t c1_int = M3508_CURRENT_FLOAT_TO_INT(c1);
	uint16_t c2_int = M3508_CURRENT_FLOAT_TO_INT(c2);
	uint16_t c3_int = M3508_CURRENT_FLOAT_TO_INT(c3);
	uint16_t c4_int = M3508_CURRENT_FLOAT_TO_INT(c4);

	uint8_t data[8] = {0};
	data[0] = (c1_int >> 8) & 0xFF;
	data[1] = c1_int & 0xFF;
	data[2] = (c2_int >> 8) & 0xFF;
	data[3] = c2_int & 0xFF;
	data[4] = (c3_int >> 8) & 0xFF;
	data[5] = c3_int & 0xFF;
	data[6] = (c4_int >> 8) & 0xFF;
	data[7] = c4_int & 0xFF;

	return can_transmit(&hfdcan1, 0x200, CAN_ID_STD, data);
}

HAL_StatusTypeDef dji3508_set_armor_vel(float vel[6]) {
	/* 0x200 + 0x1FF */
	float c1 = pid_calculate(&pid_3508v2c_5, vel[0], dji3508_5.vel);
	float c2 = pid_calculate(&pid_3508v2c_6, vel[1], dji3508_6.vel);
	float c3 = pid_calculate(&pid_3508v2c_7, vel[2], dji3508_7.vel);
	float c4 = pid_calculate(&pid_3508v2c_8, vel[3], dji3508_8.vel);
	float c5 = pid_calculate(&pid_3508v2c_9, vel[4], dji3508_9.vel);
	float c6 = pid_calculate(&pid_3508v2c_10, vel[5], dji3508_10.vel);

	uint16_t c1_int = M3508_CURRENT_FLOAT_TO_INT(c1);
	uint16_t c2_int = M3508_CURRENT_FLOAT_TO_INT(c2);
	uint16_t c3_int = M3508_CURRENT_FLOAT_TO_INT(c3);
	uint16_t c4_int = M3508_CURRENT_FLOAT_TO_INT(c4);
	uint16_t c5_int = M3508_CURRENT_FLOAT_TO_INT(c5);
	uint16_t c6_int = M3508_CURRENT_FLOAT_TO_INT(c6);

	uint8_t data1[8] = {0}, data2[8] = {0};
	data1[0] = (c1_int >> 8) & 0xFF;
	data1[1] = c1_int & 0xFF;
	data1[2] = (c2_int >> 8) & 0xFF;
	data1[3] = c2_int & 0xFF;
	data1[4] = (c3_int >> 8) & 0xFF;
	data1[5] = c3_int & 0xFF;
	data1[6] = (c4_int >> 8) & 0xFF;
	data1[7] = c4_int & 0xFF;
	data2[0] = (c5_int >> 8) & 0xFF;
	data2[1] = c5_int & 0xFF;
	data2[2] = (c6_int >> 8) & 0xFF;
	data2[3] = c6_int & 0xFF;

	uint8_t ret1 = can_transmit(&hfdcan2, 0x200, CAN_ID_STD, data1);
	uint8_t ret2 = can_transmit(&hfdcan2, 0x1FF, CAN_ID_STD, data2);
	
	if (ret1 == HAL_OK && ret2 == HAL_OK) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef dji6020_set_vel(float vel[4])
{
	/* 0x1FF */

	float volt_1 =
	    pid_calculate(&pid_6020v2v_1, vel[0], dji6020_1.vel) + 0.45 * vel[0]; /* 0.45: fore feedback */
	float volt_2 =
	    pid_calculate(&pid_6020v2v_2, vel[1], dji6020_2.vel) + 0.5 * vel[1];
	float volt_3 =
	    pid_calculate(&pid_6020v2v_3, vel[2], dji6020_3.vel) + 0.5 * vel[2];
	float volt_4 =
	    pid_calculate(&pid_6020v2v_4, vel[3], dji6020_4.vel) + 0.45 * vel[3];

	uint16_t volt_int_1 = GM6020_VOLTAGE_FLOAT_TO_INT(volt_1);
	uint16_t volt_int_2 = GM6020_VOLTAGE_FLOAT_TO_INT(volt_2);
	uint16_t volt_int_3 = GM6020_VOLTAGE_FLOAT_TO_INT(volt_3);
	uint16_t volt_int_4 = GM6020_VOLTAGE_FLOAT_TO_INT(volt_4);

	uint8_t data[8] = {0};
	data[0] = (volt_int_1 >> 8) & 0xFF;
	data[1] = volt_int_1 & 0xFF;
	data[2] = (volt_int_2 >> 8) & 0xFF;
	data[3] = volt_int_2 & 0xFF;
	data[4] = (volt_int_3 >> 8) & 0xFF;
	data[5] = volt_int_3 & 0xFF;
	data[6] = (volt_int_4 >> 8) & 0xFF;
	data[7] = volt_int_4 & 0xFF;

	return can_transmit(&hfdcan1, 0x1FF, CAN_ID_STD, data);
}

struct ramp_pos {
	float kp;
	float step;
	float smoothed_ref;
	float out_limit;
};

static struct ramp_pos pos_control[4] = {
	{.kp = 5, .step = 0.5, .smoothed_ref = 0, .out_limit = 15},
	{.kp = 7.5, .step = 0.6, .smoothed_ref = 0, .out_limit = 15},
	{.kp = 7, .step = 0.5, .smoothed_ref = 0, .out_limit = 15},
	{.kp = 5.5, .step = 0.4, .smoothed_ref = 0, .out_limit = 15}
};

/* pos1 ... pos4 should within - pi ~ pi */
HAL_StatusTypeDef dji6020_set_pos(float pos[4])
{
	debug_motor++;
	static float measure[4], vel_cmd[4];
	measure[0] = dji_get_pos(&dji6020_1, GM6020_ANGLE_OFFSET_1);
	measure[1] = dji_get_pos(&dji6020_2, GM6020_ANGLE_OFFSET_2);
	measure[2] = dji_get_pos(&dji6020_3, GM6020_ANGLE_OFFSET_3);
	measure[3] = dji_get_pos(&dji6020_4, GM6020_ANGLE_OFFSET_4);

	for (int i = 0; i < 4; i++) {
		/* make sure that smoothed_ref is within - pi ~ pi initially */
		static volatile float ref_diff = 0, vel_out = 0;
		pos[i] = update_pos_ref(pos[i], measure[i]);
		pos_control[i].smoothed_ref = update_pos_ref(pos_control[i].smoothed_ref, measure[i]);
		ref_diff = pos[i] - pos_control[i].smoothed_ref;
		if (ref_diff > pos_control[i].step) {
			pos_control[i].smoothed_ref += pos_control[i].step;
		} else if (ref_diff < - pos_control[i].step) {
			pos_control[i].smoothed_ref -= pos_control[i].step;
		} else {
			pos_control[i].smoothed_ref = pos[i];
		}

		pos_control[i].smoothed_ref = update_pos_ref(pos_control[i].smoothed_ref, measure[i]);
		vel_out = pos_control[i].kp * (pos_control[i].smoothed_ref - measure[i]);
		if (vel_out > pos_control[i].out_limit) {
			vel_cmd[i] = pos_control[i].out_limit;
		} else if (vel_out < - pos_control[i].out_limit) {
			vel_cmd[i] = - pos_control[i].out_limit;
		} else {
			vel_cmd[i] = vel_out;
		}
	}

	return dji6020_set_vel(vel_cmd);
}
