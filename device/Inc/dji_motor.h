#ifndef MOTOR_H__
#define MOTOR_H__

#include <stdint.h>
#include "main.h"

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

    float pos;    // rad
    float vel; // rad/s
    float cur;  // A

    enum motor_type type;
};

void dji_motor_interpret(uint8_t *rx_buff, struct motor_info *motor);
HAL_StatusTypeDef dji3508_set_chassis_vel(float v1, float v2, float v3, float v4);
HAL_StatusTypeDef dji6020_set_vel(float v1, float v2, float v3, float v4);

float update_pos(struct motor_info *motor, int offset);
HAL_StatusTypeDef dji6020_set_pos(float pos1, float pos2, float pos3, float pos4);


extern struct motor_info dji3508_1;
extern struct motor_info dji3508_2;
extern struct motor_info dji3508_3;
extern struct motor_info dji3508_4;
extern struct motor_info dji6020_1;
extern struct motor_info dji6020_2;
extern struct motor_info dji6020_3;
extern struct motor_info dji6020_4;

#endif //MOTOR_H__
