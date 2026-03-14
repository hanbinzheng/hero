#include "dji_motor.h"
#include "bsp_fdcan.h"
#include "pid.h"

#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f)
// GM6020
#define GM6020_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 3.0f)) // -3A~0~3A, -16384~0~16384
#define GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 3.0f / 16384.0f)
#define GM6020_VOLTAGE_FLOAT_TO_INT(value) ((int16_t)((value) * 25000.0f / 24.0f)) // -24v~0~24v, -25000~0~25000
// #define GM6020_VOLTAGE_FLOAT_TO_INT(value) (int16_t)((value)) // -25000~0~25000
#define GM6020_LINEAR_RATE (0.5f)

// M3508
#define M3508_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 20.0f)) // -20A~0~20A, -16384~0~16384
#define M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 20.0f / 16384.0f)
#define M3508_REDUC_RATE (3591.0f / 187.0f)

// M2006
#define M2006_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 10000.0f / 10.0f)) // -10A~0~10A, -10000~0~10000
#define M2006_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 10.0f / 10000.0f)


/*
 **************************************************************************
 * Motor Informations
 **************************************************************************
 */
struct motor_info dji3508_1 = {.type = M3508};
struct motor_info dji3508_2 = {.type = M3508};
struct motor_info dji3508_3 = {.type = M3508};
struct motor_info dji3508_4 = {.type = M3508};
struct motor_info dji6020_1 = {.type = GM6020};
struct motor_info dji6020_2 = {.type = GM6020};
struct motor_info dji6020_3 = {.type = GM6020};
struct motor_info dji6020_4 = {.type = GM6020};

static struct pid_info pid_3508v2c_1 = {
	.kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_2 = {
	.kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_3 = {
	.kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};
static struct pid_info pid_3508v2c_4 = {
	.kp = 0.045f, .ki = 0.008, .kd = 0, .i_limit = 0.5, .out_limit = 20};

static struct pid_info pid_6020v2v_1 = {
	.kp = 1.25f, .ki = 0.5f, .kd = 0, .i_limit = 0.75f, .out_limit = 20};
static struct pid_info pid_6020v2v_2 = {
	.kp = 1.4f, .ki = 0.2f, .kd = 0, .i_limit = 0.5f, .out_limit = 20};
static struct pid_info pid_6020v2v_3 = {
	.kp = 1.25f, .ki = 0.1f, .kd = 0, .i_limit = 0.5f, .out_limit = 20};
static struct pid_info pid_6020v2v_4 = {
	.kp = 1.0f, .ki = 0.1f, .kd = 0, .i_limit = 0.5f, .out_limit = 20};


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
    switch (motor->type)
    {
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


HAL_StatusTypeDef dji3508_set_chassis_vel(float v1, float v2, float v3, float v4) {
	/* 0x200 */
	float c1 = pid_calculate(&pid_3508v2c_1, v1 * M3508_REDUC_RATE, dji3508_1.vel);
	float c2 = pid_calculate(&pid_3508v2c_2, v2 * M3508_REDUC_RATE, dji3508_2.vel);
	float c3 = pid_calculate(&pid_3508v2c_3, v3 * M3508_REDUC_RATE, dji3508_3.vel);
	float c4 = pid_calculate(&pid_3508v2c_4, v4 * M3508_REDUC_RATE, dji3508_4.vel);

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

HAL_StatusTypeDef dji6020_set_vel(float v1, float v2, float v3, float v4) {
	/* 0x1FF */
	
	float volt_1 = pid_calculate(&pid_6020v2v_1, v1, dji6020_1.vel) + GM6020_LINEAR_RATE * v1;
	float volt_2 = pid_calculate(&pid_6020v2v_2, v2, dji6020_2.vel) + GM6020_LINEAR_RATE * v2;
	float volt_3 = pid_calculate(&pid_6020v2v_3, v3, dji6020_3.vel) + GM6020_LINEAR_RATE * v3;
	float volt_4 = pid_calculate(&pid_6020v2v_4, v4, dji6020_4.vel) + GM6020_LINEAR_RATE * v4;

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