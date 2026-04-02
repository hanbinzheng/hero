#ifndef VISION_H_
#define VISION_H_

#include <stdint.h>

#pragma pack(push, 1)
struct vision_raw_data {
    uint8_t _0x55; /* dummy */
    uint8_t length;
    uint8_t _0x00_1; /* dummy */
    uint8_t _0x00_2; /* dummy */
    uint8_t crc8;
    uint8_t _0x01; /* dummy */
    uint8_t _0x00_3; /* dummy */

    uint8_t _0xAA; /* dummy */
    float ang_pitch;
    float ang_yaw;
    float vel_yaw;
    int16_t can_shoot;
    int16_t move_state;
    int16_t if_get;
    uint8_t enemy_kind;
    int16_t enemy_x; /* cm */
    int16_t enemy_y; /* cm */
    uint8_t _0xA5; /* dummy */
    
    uint16_t crc16;
};
#pragma pack(pop)

struct vision_data {
    float tick;
    float ang_yaw;
    float ang_pitch;
    float dx; /* cm */
    float dy; /* cm */
    int16_t can_shoot;
};

void vision_send_feedback(void);

extern struct vision_data vision_data;

#endif
