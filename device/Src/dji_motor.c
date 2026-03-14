#include "dji_motor.h"
#include "bsp_fdcan.h"

#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f)
// GM6020
#define GM6020_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 3.0f)) // -3A~0~3A, -16384~0~16384
#define GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 3.0f / 16384.0f)
#define GM6020_VOLTAGE_FLOAT_TO_INT(value) ((int16_t)((value) * 25000.0f / 24.0f)) // -24v~0~24v, -25000~0~25000
// #define GM6020_VOLTAGE_FLOAT_TO_INT(value) (int16_t)((value)) // -25000~0~25000

// M3508
#define M3508_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 20.0f)) // -20A~0~20A, -16384~0~16384
#define M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 20.0f / 16384.0f)

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