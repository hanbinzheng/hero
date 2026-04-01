#ifndef SUPER_CAP_H_
#define SUPER_CAP_H_

#include <stdint.h>

#define SUPER_CAP_VOLTAGE_SCALE (1.0f / 1000.0f)
#define SUPER_CAP_ENERGY_SCALE (1.0f / 29.78863f)

struct super_cap_data {
    uint16_t raw_voltage;
    uint16_t raw_energy;
    uint8_t state;

    float vlotage;
    float energy;
};

enum super_cap_state {
    FSM_OK = 0,
    FSM_CAP_OUT_FAULT = 1,
    FSM_SHORT_A_FAULT = 2,
    FSM_SHORT_B_FAULT = 4,
    FSM_OV_A_FAULT = 8,
    FSM_OV_B_FAULT = 16,
    FSM_PWR_OFF_FAULT = 32,
    FSM_CAP_ENERGY_LOW = 64,
};

void super_cap_interpret(uint8_t *rx_buff, struct super_cap_data *super_cap_data);

extern struct super_cap_data super_cap_data;
#endif