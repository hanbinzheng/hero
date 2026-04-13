#include "vision.h"
#include "arm_math.h"
#include "crc.h"
#include "imu.h"
#include "referee.h"
#include "usbd_cdc_if.h"
#include "vt03.h"

#ifndef _PI_OVER_180_
#define _PI_OVER_180_ (3.1415926535897932384f / 180.0f)
#endif

struct vision_data vision_data;

static uint8_t tx_buff[64];
void vision_send_feedback(void)
{
	uint8_t *buff = tx_buff;
	buff[0] = 0xAA;

	/* determine the color of enemy based on own id, 1 red, 2 blue */
	buff[1] = (referee_info.robot_status.robot_id < 0x100) ? 2 : 1;

	*(float *)(&buff[2]) = imu_data.pos_yaw;   /* 4 bytes */
	*(float *)(&buff[6]) = imu_data.pos_pitch; /* 4 bytes */

	buff[10] = 0x00; /* grade */

	/* crc check sum */
	uint16_t crc16 = get_crc16_check_sum(buff, 11);
	buff[11] = (uint8_t)(crc16 & 0xFF);
	buff[12] = (uint8_t)(crc16 >> 8);

	/* transmit via usb */
	CDC_Transmit_HS(buff, 13);
}

static void vision_receiption_handler(uint8_t *buff, uint32_t *len)
{
	int8_t length = (int8_t)(*len);

	if (length != 34) /* vision data is exactly 34 bytes */
		return;

	struct vision_raw_data *vision_raw_data = (struct vision_raw_data *)buff;

	/* check header */
	if (vision_raw_data->_0x55 != 0x55 || vision_raw_data->_0xAA != 0xAA ||
	    vision_raw_data->crc8 != get_crc8_check_sum(buff, 4)) {
		return;
	}

	/* get data */
	vision_data.ang_yaw = -1.0f * vision_raw_data->ang_yaw;
	vision_data.ang_pitch = vision_raw_data->ang_pitch;
	vision_data.can_shoot = vision_raw_data->can_shoot;

	float distance;
	arm_sqrt_f32(vision_raw_data->enemy_x * vision_raw_data->enemy_x +
			 vision_raw_data->enemy_y * vision_raw_data->enemy_y,
		     &distance);

	vision_data.dx = distance * (vision_data.ang_yaw * _PI_OVER_180_);
	vision_data.dy = distance * (vision_data.ang_pitch * _PI_OVER_180_);
}

void cdc_receive_handler(uint8_t *buff, uint32_t *len)
{
	vision_receiption_handler(buff, len);
}