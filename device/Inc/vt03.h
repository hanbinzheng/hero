#ifndef VT_H_
#define VT_H_

/* revised from Yaofang Ji https://github.com/Froze23n/ */

#include "stdint.h"

#define MODE_C 0
#define MODE_N 1
#define MODE_S 2

#pragma pack(push, 1)
struct vt03_raw_data {
	/*********************************************
	 * start of the frame: 2 bytes
	 *********************************************/
	uint8_t sof_1; /* 0xA9 */
	uint8_t sof_2; /* 0x53 */

	/*********************************************
	 * remote control data frame: 8 bytes
	 *********************************************/
	union {
		uint64_t raw;
		struct {
			uint64_t ch_0 : 11;
			uint64_t ch_1 : 11;
			uint64_t ch_2 : 11;
			uint64_t ch_3 : 11;
			uint64_t mode_sw : 2;
			uint64_t pause : 1;
			uint64_t fn_1 : 1;
			uint64_t fn_2 : 1;
			uint64_t wheel : 11;
			uint64_t trigger : 1;
			uint64_t _unused : 3;
		} bit;
	} rc;

	/*********************************************
	 * mouse frame: 7 byte
	 *********************************************/
	union {
		uint8_t raw[7];
		struct {
			int16_t mouse_x;
			int16_t mouse_y;
			int16_t mouse_z;

			uint8_t mouse_left : 2;
			uint8_t mouse_right : 2;
			uint8_t mouse_middle : 2;
			uint8_t _unused : 2;
		} bit;
	} mouse;

	/*********************************************
	 * keyboard frame: 2 bytes
	 *********************************************/
	union {
		uint16_t raw;
		struct {
			uint16_t W : 1;
			uint16_t S : 1;
			uint16_t A : 1;
			uint16_t D : 1;
			uint16_t SHIFT : 1;
			uint16_t CTRL : 1;
			uint16_t Q : 1;
			uint16_t E : 1;
			uint16_t R : 1;
			uint16_t F : 1;
			uint16_t G : 1;
			uint16_t Z : 1;
			uint16_t X : 1;
			uint16_t C : 1;
			uint16_t V : 1;
			uint16_t B : 1;
		} bit;
	} keyboard;

	/*********************************************
	 * CRC verification
	 *********************************************/
	uint16_t crc16;
};
#pragma pack(pop)

struct vt03_data {
	float ls_x;
	float ls_y;
	float rs_x;
	float rs_y;
	float wheel;
	uint8_t mode_sw;
	uint8_t pause;
	uint8_t l_fn;
	uint8_t r_fn;
	uint8_t trigger;
};

extern struct vt03_raw_data vt03_raw_data;
extern struct vt03_data vt03_data;

#endif /* VT_H_ */
