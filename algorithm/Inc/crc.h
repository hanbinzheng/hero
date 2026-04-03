#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

uint8_t get_crc8_check_sum(uint8_t *p_msg, uint32_t len);
uint16_t get_crc16_check_sum(uint8_t *p_msg, uint32_t len);

#endif /* CRC_H_ */