#include "super_cap.h"
#include "bsp_fdcan.h"
#include "referee.h"

struct super_cap_data super_cap_data;

int volatile super_cap_debug = 0;

/* cannot be placed directly into super_cap_interpret */
static void update_super_cap_cmd(void)
{
    static uint8_t tx_buff[3];

    tx_buff[0] = (uint8_t) referee_info.robot_status.chassis_power_limit;
    tx_buff[1] = (uint8_t) referee_info.power_heat_data.buffer_energy;
    tx_buff[2] = 0;

    can_transmit(&hfdcan1, 0x498, CAN_ID_STD, tx_buff);
}

void super_cap_interpret(uint8_t *rx_buff, struct super_cap_data *super_cap_data)
{
        super_cap_data->raw_voltage = (rx_buff[0] << 8) | rx_buff[1];
        super_cap_data->raw_energy = (rx_buff[2] << 8) | rx_buff[3];
        super_cap_data->state = rx_buff[4];

        super_cap_data->vlotage = ((float)super_cap_data->raw_voltage) * SUPER_CAP_VOLTAGE_SCALE;
        super_cap_data->energy = ((float)super_cap_data->raw_energy) * SUPER_CAP_ENERGY_SCALE;
}
