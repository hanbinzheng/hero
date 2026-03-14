#include "dbus.h"
#include "main.h"
#include <stdint.h>

/*
 * some macros defined in dbus.h
 * DBUS_FRAME_LENGTH 18
 * DBUS_RX_BUF_NUM 36
 */
// reference:
// 1. https://zhuanlan.zhihu.com/p/4218673539
// 2.
// https://community.st.com/t5/stm32-mcus/dma-is-not-working-on-stm32h7-devices/ta-p/49498

#define CHANNEL_OFFSET 1024  // channel offset
#define CHANNEL_RATIO 660.0f // normalize channel data

/*
**************************************************************************
* global variables
**************************************************************************
*/
// DMA1 and DMA2 cannot access DTCM for STM32H7 series, here change to D2SRAM
uint32_t dbus_tick;
struct dbus_data dbus_data;

static void dbus_data_interpret(uint8_t *buff, struct dbus_data *dbus_data)
{
	// get tick to feed the dog
	dbus_tick = HAL_GetTick();

	dbus_data->rs_y =
	    (((buff[0] | (buff[1] << 8)) & 0x07FF) - CHANNEL_OFFSET) / CHANNEL_RATIO;
	dbus_data->rs_x =
	    ((((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF) - CHANNEL_OFFSET) /
	    CHANNEL_RATIO;
	dbus_data->ls_y =
	    ((((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF) -
	     CHANNEL_OFFSET) /
	    CHANNEL_RATIO;
	dbus_data->ls_x =
	    ((((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF) - CHANNEL_OFFSET) /
	    CHANNEL_RATIO;

	dbus_data->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	dbus_data->sw2 = (buff[5] >> 4) & 0x0003;

	dbus_data->wheel = buff[16] | (buff[17] << 8); /* 364~1024~1684 */

	dbus_data->mouse.x = buff[6] | (buff[7] << 8);
	dbus_data->mouse.y = buff[8] | (buff[9] << 8);
	dbus_data->mouse.z = buff[10] | (buff[11] << 8);
	dbus_data->mouse.l = buff[12];
	dbus_data->mouse.r = buff[13];

	dbus_data->keyboard.key_code = buff[14] | (buff[15] << 8);
}

/* rewrite weak function in bsp_usart.c */
void usart5_data_interpret(uint8_t *rx_buff)
{
	dbus_data_interpret(rx_buff, &dbus_data);
}
