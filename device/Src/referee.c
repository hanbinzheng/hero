#include "referee.h"
#include "bsp_usart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "super_cap.h"

#define SOF 0xA5
struct referee_info referee_info;

/* crc8 and crc16 code */
static const unsigned char crc8_init = 0xFF;
static const uint16_t crc16_init = 0xFFFF;
static const unsigned char crc8_table[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20,
        0xa3, 0xfd, 0x1f, 0x41, 0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
        0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 0x7d, 0x9f, 0xc1,
        0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e,
        0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
        0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 0x39,
        0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45,
        0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
        0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 0x8c, 0xd2, 0x30, 0x6e,
        0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31,
        0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
        0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 0xd0,
        0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea,
        0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
        0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b,
        0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54,
        0xd7, 0x89, 0x6b, 0x35,
};
static const uint16_t w_crc_table[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48,
        0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108,
        0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb,
        0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
        0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e,
        0xfae7, 0xc87c, 0xd9f5, 0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e,
        0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd,
        0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285,
        0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44,
        0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 0x6306, 0x728f, 0x4014,
        0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
        0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3,
        0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862,
        0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e,
        0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1,
        0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483,
        0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50,
        0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
        0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7,
        0x6e6e, 0x5cf5, 0x4d7c, 0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1,
        0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72,
        0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e,
        0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf,
        0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 0xf78f, 0xe606, 0xd49d,
        0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
        0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
};

static uint8_t get_crc8_check_sum(uint8_t *pch_message, uint32_t dw_length) {
        uint8_t uc_idx;
        uint8_t uc_crc8 = crc8_init; /* 0xFF */
        while (dw_length--) {
                uc_idx = uc_crc8 ^ (*pch_message++);
                uc_crc8 = crc8_table[uc_idx];
        }
        return (uc_crc8);
}

static uint16_t get_crc16_check_sum(uint8_t *pch_message, uint32_t dw_length) {
        uint16_t w_crc = crc16_init;
        uint8_t ch_data;
        if (pch_message == NULL) {
                return 0xFFFF;
        }
        while (dw_length--) {
                ch_data = *pch_message++;
                (w_crc) = ((uint16_t)(w_crc) >> 8) ^
                         w_crc_table[((uint16_t)(w_crc) ^ (uint16_t)(ch_data)) &
                                    0x00ff];
        }
        return w_crc;
}

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
        if(total_len > UART10_RX_BUFF_LEN)
                return;
    
        while(total_len != 0)
        {
                /* Invalid frame header, stop processing */
                if(SOF != rx_buff[0])
                        return;

                /* length of data frame */
                uint16_t data_length = rx_buff[1] | (rx_buff[2] << 8);

                static uint8_t seq = 0xFF;
                if(rx_buff[3] != seq + 1) {
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
                if(crc16 != get_crc16_check_sum(rx_buff, data_length + 7)){
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
                        memcpy(&(referee->projectile_allowance), data, sizeof(struct projectile_allowance));
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

__attribute__((section(".D1_SECTION"))) static uint8_t tx_buff[128] = {0};
static uint32_t cap = 0;
uint32_t aim_x = 960;
uint32_t aim_y = 540;
HAL_StatusTypeDef referee_ui_update(void){
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
        uint16_t receiver_id = (sender_id < 100) ? 1 : 101; /* red hero: 1, bulue hero: 101 */
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
        figure->layer = 0; /* figure layer 0 */
        figure->color = 0; /* color of our side (red or blue) */
        /* line: 1920 x 1080 */
        figure->width = 20; /* line width */
        figure->start_x = 680;
        figure->start_y = 250;
        figure->details_d = figure->start_x + cap; /* x coordinate of line end */
        figure->details_e = figure->start_y; /* y coordinate of line end */

        /* byte 28 - 42: the second figure */
        figure = (struct interaction_figure *)&tx_buff[28];
        figure->operate_type = (seq % 32) ? 2:1 ;
        figure->figure_type = 1; /* rectangle */
        figure->layer = 1;
        figure->color = 1; /* yellow */
        figure->width = 5;
        figure->start_x = aim_x - 60; /* x coordinate of starting vertex */
        figure->start_y = aim_y - 40; /* y coordinate of starting vertex */
        figure->details_d = aim_x + 60; /* x coordinate of opposite vertex */
        figure->details_e = aim_y + 40; /* y coordinate of opposite vertex */
    
        /* frame tail: CRC16 of data section */
        uint16_t crc16 = get_crc16_check_sum(tx_buff, data_length + 5 + 2); /* 5 header + 2 cmd_id */
        tx_buff[data_length + 7] = (uint8_t)(crc16 & 0xFF);
        tx_buff[data_length + 8] = (uint8_t)(crc16 >> 8);
    
        /* frame_header: 5, cmd_id: 2, data: n, frame_tail: 2 */
        return HAL_UART_Transmit_DMA(&huart10, tx_buff, 5 + 2 + data_length + 2);
}

void uart10_data_interpret(uint8_t *rx_buff, uint16_t received_len)
{
        referee_data_interpret(rx_buff, received_len, &referee_info);
}
