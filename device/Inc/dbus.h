#ifndef DBUS_H_
#define DBUS_H_

#include "stdint.h"

#define SW_UP 0x01
#define SW_MID 0x03
#define SW_DOWN 0x02
#define DBUS_OFFLINE_TICK 200 /* ms */

#define DBUS_FRAME_LENGTH 18 /* 18 byte a data frame */
#define DBUS_RX_BUF_NUM 36   /* double buffer dma */

struct dbus_data {
	/* Normalize channel value: -660 ~ 660 -> -1 ~ 1 */
	float ls_x;
	float ls_y;
	float rs_x;
	float rs_y;

	/* Switch state: SW_UP, SW_MID, SW_DOWN */
	uint8_t sw1;
	uint8_t sw2;

	/* Left wheel */
	int16_t wheel;

	struct {
		/* movement */
		int16_t x;
		int16_t y;
		int16_t z; /* not in use */

		/* press: 1, release: 0 */
		uint8_t l;
		uint8_t r;
	} mouse;

	/* Keyboard */
	/**********************************************************************************
	 * keyboard: 15   14   13   12   11   10   9   8   7   6     5     4   3
	 *   2   1 V    C    X	 Z    G    F   R   E   Q  CTRL SHIFT   D   A   S
	 *   W
	 ************************************************************************************/
	union {
		uint16_t key_code;
		struct {
			uint16_t w : 1;
			uint16_t s : 1;
			uint16_t a : 1;
			uint16_t d : 1;
			uint16_t shift : 1;
			uint16_t ctrl : 1;
			uint16_t q : 1;
			uint16_t e : 1;
			uint16_t r : 1;
			uint16_t f : 1;
			uint16_t g : 1;
			uint16_t z : 1;
			uint16_t x : 1;
			uint16_t c : 1;
			uint16_t v : 1;
			uint16_t b : 1;
		} key_bit;
	} keyboard;
};

extern struct dbus_data dbus_data;
extern uint32_t dbus_tick;

#endif // DBUS_H_
