#include "vt03.h"
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include "iwdg.h"
#include "crc.h"

#include <string.h>

int freq_count = 0;
struct vt03_data vt03_data;

static void vt03_data_interpret(uint8_t *rx_buff, int total_len,
	struct vt03_data *vt03_data)
{
	if (total_len > UART10_RX_BUFF_LEN) {
		return; /* too long */
	}

	while (total_len >= 0) {
		if (rx_buff[0] != 0xA9 || rx_buff[1] != 0x53) {
			return; /* sof check fails */
		}

		/* check crc16 */
		uint16_t crc16 = get_crc16_check_sum(rx_buff, VT03_FRAME_LENGTH - 2);
		if (crc16 == (rx_buff[VT03_FRAME_LENGTH - 2] | (rx_buff[VT03_FRAME_LENGTH - 1] << 8))) {
			/* feed the dog and enable control */
			HAL_IWDG_Refresh(&hiwdg1);
			control_ready = 1;
			freq_count++;

			struct vt03_raw_data *raw_data = (struct vt03_raw_data *)rx_buff;

			/* ch_0 ~ ch_4: mapping 364 ~ 1684 to -1 ~ 1 */
			vt03_data->ls_x = ((float)((raw_data->rc).bit.ch_2 - 1024)) / 660.0f;
			vt03_data->ls_y = -((float)((raw_data->rc).bit.ch_3 - 1024)) / 660.0f;
			vt03_data->rs_x = ((float)((raw_data->rc).bit.ch_1 - 1024)) / 660.0f;
			vt03_data->rs_y = -((float)((raw_data->rc).bit.ch_0 - 1024)) / 660.0f;

			/* wheel: mapping 364 ~ 1684 to -1 ~ 1 */
			vt03_data->wheel = ((float)((raw_data->rc).bit.wheel - 1024)) / 660.0f;

			/* mode c: 0, mode n: 1, mode s: 2 */
			vt03_data->mode_sw = (raw_data->rc).bit.mode_sw;

			/* 0: unpressed, 1: pressed */
			vt03_data->pause = (raw_data->rc).bit.pause;
			vt03_data->l_fn = (raw_data->rc).bit.fn_1;
			vt03_data->r_fn = (raw_data->rc).bit.fn_2;
			vt03_data->trigger = (raw_data->rc).bit.trigger;

			/* mouse data */
			vt03_data->mouse_left = (raw_data->mouse).bit.mouse_left;
			vt03_data->mouse_right = (raw_data->mouse).bit.mouse_right;
			vt03_data->mouse_middle = (raw_data->mouse).bit.mouse_middle;
			/* map -32768 ~ 32768 to a specific scale */
			vt03_data->mouse_x = (float)(raw_data->mouse).bit.mouse_x / 327.68f;
			vt03_data->mouse_y = (float)(raw_data->mouse).bit.mouse_y / 327.68f;
			vt03_data->mouse_z = (float)(raw_data->mouse).bit.mouse_z / 327.68f;

			/* keyboard data */
			(vt03_data->keyboard).raw = (raw_data->keyboard).raw;
		} else {
			/* crc16 failure logic */
		}


		rx_buff += VT03_FRAME_LENGTH;
		total_len -= VT03_FRAME_LENGTH;
	}
}

void uart7_data_interpret(uint8_t *rx_buff, uint16_t received_len)
{
		vt03_data_interpret(rx_buff, received_len, &vt03_data);
}