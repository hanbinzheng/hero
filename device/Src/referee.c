#include "referee.h"
#include "bsp_usart.h"
#include "crc.h"
#include "super_cap.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SOF 0xA5

struct referee_info referee_info;

/*-------------------- data interpret function -------------------------*/

/**
 * @brief Parse referee system data from received buffer
 * @param rx_buff Pointer to received data buffer
 * @param total_len Total length of received data
 * @param referee Pointer to referee info structure to store parsed data
 * @note The referee protocol format:
 *       [SOF][Data Len][Seq][CRC8][CMD ID][Data][CRC16]
 *        1 byte + 2 bytes + 1 byte + 1 byte + 2 bytes + n bytes + 2 bytes
 *       frame_header: [SOF][Data Len][Seq][CRC8]
 */
static void referee_data_interpret(uint8_t *rx_buff, uint16_t total_len,
				   struct referee_info *referee)
{
	/* Check for buffer overflow */
	if (total_len > UART10_RX_BUFF_LEN)
		return;

	while (total_len != 0) {
		/* Invalid frame header, stop processing */
		if (SOF != rx_buff[0])
			return;

		/* length of data frame */
		uint16_t data_length = rx_buff[1] | (rx_buff[2] << 8);

		static uint8_t seq = 0xFF;
		if (rx_buff[3] != seq + 1) {
			/* Sequence error detected, may need error handling */
		}
		seq = rx_buff[3]; /* Update sequence number */

		/* Validate header CRC8 (SOF + Data Len + Seq) */
		uint8_t crc8 = rx_buff[4];
		if (crc8 != get_crc8_check_sum(rx_buff, 4)) {
			return; /* CRC8 error, discard entire packet */
		}

		uint16_t cmd_id = rx_buff[5] | (rx_buff[6] << 8);
		uint8_t *data = rx_buff + 7;
		uint16_t crc16 = data[data_length] | (data[data_length + 1] << 8);

		/* Validate data CRC16 */
		if (crc16 != get_crc16_check_sum(rx_buff, data_length + 7)) {
			/* CRC16 error, skip this packet and continue to next */
			rx_buff += (5 + 2 + data_length + 2); /* 5 header + 2 cmd_id + 2 CRC16 */
			total_len -= (5 + 2 + data_length + 2);
			continue;
		}

		switch (cmd_id) {
		case ID_GAME_STATUS:
			memcpy(&(referee->game_status), data, sizeof(struct game_status));
			break;
		case ID_GAME_RESULT:
			memcpy(&(referee->game_result), data, sizeof(struct game_result));
			break;
		case ID_GAME_ROBOT_HP:
			memcpy(&(referee->game_robot_hp), data, sizeof(struct game_robot_hp));
			break;
		case ID_EVENT_DATA:
			memcpy(&(referee->event_data), data, sizeof(struct event_data));
			break;
		case ID_REFEREE_WARNING:
			memcpy(&(referee->referee_warning), data, sizeof(struct referee_warning));
			break;
		case ID_DART_INFO:
			memcpy(&(referee->dart_info), data, sizeof(struct dart_info));
			break;
		case ID_ROBOT_STATUS:
			memcpy(&(referee->robot_status), data, sizeof(struct robot_status));
			break;
		case ID_POWER_HEAT_DATA:
			memcpy(&(referee->power_heat_data), data, sizeof(struct power_heat_data));
			break;
		case ID_ROBOT_POS:
			memcpy(&(referee->robot_pos), data, sizeof(struct robot_pos));
			break;
		case ID_BUFF:
			memcpy(&(referee->buff), data, sizeof(struct buff));
			break;
		case ID_HURT_DATA:
			memcpy(&(referee->hurt_data), data, sizeof(struct hurt_data));
			break;
		case ID_SHOOT_DATA:
			memcpy(&(referee->shoot_data), data, sizeof(struct shoot_data));
			break;
		case ID_PROJECTILE_ALLOWANCE:
			memcpy(&(referee->projectile_allowance), data,
			       sizeof(struct projectile_allowance));
			break;
		case ID_RFID_STATUS:
			memcpy(&(referee->rfid_status), data, sizeof(struct rfid_status));
			break;
		default:
			break;
		}

		/* Move to next packet and continue processing */
		rx_buff += (data_length + 9); /* Advance by: 5 header + 2 cmd_id + 2 CRC16 */
		total_len -= (data_length + 9);
	}
}

__attribute__((section(".dma12_buffer"))) static uint8_t tx_buff[128] = {0};
static uint32_t cap = 0;
uint32_t aim_x = 960;
uint32_t aim_y = 540;
HAL_StatusTypeDef referee_ui_update(void)
{
	/* update figure info */
	cap = (uint32_t)(super_cap_data.energy * 0.25f);
	aim_x = 860 + rand() % 200;
	aim_y = 440 + rand() % 200;

	/* communication:
	 * frame header (5 bytes: SOF, data length (n), deq, CRC8)
	 * command id (2 bytes)
	 * data (n bytes)
	 * frame tail (2 bytes: CRC16)
	 */

	/* byte 0 - 4: frame header, 5 bytes */
	tx_buff[0] = SOF;
	uint16_t data_length = 36; /* 36 = 6 (header) + 15 x 2 (figure x 2) */
	tx_buff[1] = (uint8_t)(data_length & 0xFF);
	tx_buff[2] = (uint8_t)(data_length >> 8);
	static uint8_t seq = 0; /* package sequence */
	tx_buff[3] = seq++;
	tx_buff[4] = get_crc8_check_sum(tx_buff, 4);

	/* byte 5 & 6: command id, 2 bytes */
	tx_buff[5] = (uint8_t)(ID_ROBOT_INTERACTION_DATA & 0xFF);
	tx_buff[6] = (uint8_t)(ID_ROBOT_INTERACTION_DATA >> 8);

	/* data command:
	 * data command id (2 bytes)
	 * sender id (2 bytes)
	 * receiver id (2 bytes)
	 * data content (x bytes: x <= 112)
	 */

	/* byte 7 - 12: info about command */
	uint16_t data_cmd_id = 0x102; /* draw command: 2 figures */
	tx_buff[7] = (uint8_t)(data_cmd_id & 0xFF);
	tx_buff[8] = (uint8_t)(data_cmd_id >> 8);
	/* sender id: this robot id */
	uint16_t sender_id = referee_info.robot_status.robot_id;
	tx_buff[9] = (uint8_t)(sender_id & 0xFF);
	tx_buff[10] = (uint8_t)(sender_id >> 8);
	/* receiver id: the client */
	uint16_t receiver_id = (sender_id < 0x0100) ? (sender_id + 0x0100) : (sender_id + 0x0064);
	tx_buff[11] = (uint8_t)(receiver_id & 0xFF);
	tx_buff[12] = (uint8_t)(receiver_id >> 8);

	/* byte 13 - 27: the first figure */
	struct interaction_figure *figure = (struct interaction_figure *)&tx_buff[13];
	figure->figure_name[0] = 'C';
	figure->figure_name[1] = 'A';
	figure->figure_name[2] = 'P';
	/* to prevent figure from disappearing, to adding at certain frequency */
	figure->operate_type = (seq % 32) ? 2 : 1;
	figure->figure_type = 0; /* line */
	figure->layer = 0;	 /* figure layer 0 */
	figure->color = 0;	 /* color of our side (red or blue) */
	/* line: 1920 x 1080 */
	figure->width = 20; /* line width */
	figure->start_x = 680;
	figure->start_y = 250;
	figure->details_d = figure->start_x + cap; /* x coordinate of line end */
	figure->details_e = figure->start_y;	   /* y coordinate of line end */

	/* byte 28 - 42: the second figure */
	figure = (struct interaction_figure *)&tx_buff[28];
	figure->operate_type = (seq % 32) ? 2 : 1;
	figure->figure_type = 1; /* rectangle */
	figure->layer = 1;
	figure->color = 1; /* yellow */
	figure->width = 5;
	figure->start_x = aim_x - 60;	/* x coordinate of starting vertex */
	figure->start_y = aim_y - 40;	/* y coordinate of starting vertex */
	figure->details_d = aim_x + 60; /* x coordinate of opposite vertex */
	figure->details_e = aim_y + 40; /* y coordinate of opposite vertex */

	/* frame tail: CRC16 of data section */
	uint16_t crc16 =
	    get_crc16_check_sum(tx_buff, data_length + 5 + 2); /* 5 header + 2 cmd_id */
	tx_buff[data_length + 7] = (uint8_t)(crc16 & 0xFF);
	tx_buff[data_length + 8] = (uint8_t)(crc16 >> 8);

	/* frame_header: 5, cmd_id: 2, data: n, frame_tail: 2 */
	return HAL_UART_Transmit_DMA(&huart10, tx_buff, 5 + 2 + data_length + 2);
}

void uart10_data_interpret(uint8_t *rx_buff, uint16_t received_len)
{
	referee_data_interpret(rx_buff, received_len, &referee_info);
}
