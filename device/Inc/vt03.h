//
// Created by YawFun on 25-11-20.
//

#ifndef VT_H
#define VT_H

/* ------------------------------ Include ------------------------------ */
#include "stdint.h"

/* ------------------------------ Type Definition ------------------------------ */
#pragma pack(push, 1)
struct vt03_data
{
    /*********************************************
     * 帧头 共 2 字节
     *********************************************/
    uint8_t sof_1; //0xA9
    uint8_t sof_2; //0x53

    /*********************************************
     * 遥控器数据段，共 8 字节
     *********************************************/
    union {
        uint64_t raw;
        struct {
            uint64_t ch_0       : 11;
            uint64_t ch_1       : 11;
            uint64_t ch_2       : 11;
            uint64_t ch_3       : 11;
            uint64_t mode_sw    : 2;
            uint64_t pause      : 1;
            uint64_t fn_1       : 1;
            uint64_t fn_2       : 1;
            uint64_t wheel      : 11;
            uint64_t trigger    : 1;
            uint64_t _unused    : 3;
        } bit;
    } rc;

    /*********************************************
     * 鼠标段，共 7 字节
     *********************************************/
    union {
        uint8_t raw[7];
        struct {
            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;

            uint8_t mouse_left   : 2;
            uint8_t mouse_right  : 2;
            uint8_t mouse_middle : 2;
            uint8_t _unused      : 2;
        } bit;
    } mouse;

    /*********************************************
     * 键盘按键，共 2 字节
     *********************************************/
    union {
        uint16_t raw;
        struct {
            uint16_t W      : 1;
            uint16_t S      : 1;
            uint16_t A      : 1;
            uint16_t D      : 1;
            uint16_t SHIFT  : 1;
            uint16_t CTRL   : 1;
            uint16_t Q      : 1;
            uint16_t E      : 1;
            uint16_t R      : 1;
            uint16_t F      : 1;
            uint16_t G      : 1;
            uint16_t Z      : 1;
            uint16_t X      : 1;
            uint16_t C      : 1;
            uint16_t V      : 1;
            uint16_t B      : 1;
        } bit;
    } keyboard;

    /*********************************************
     * CRC16 校验数据
     *********************************************/
    uint16_t crc16;

};
#pragma pack(pop)

//仅用于Ozone中进行数据可视化
struct debug_type {
    float ch0;
    float ch1;
    float ch2;
    float ch3;
    uint8_t mode_sw;
    uint8_t pause;
    uint8_t fn_1;
    uint8_t fn_2;
    float wheel;
    uint8_t trigger;
} ;
/* ------------------------------ Extern Global Variable ------------------------------ */
extern struct vt03_data vt03_data;
extern struct debug_type debug;

#endif //VT2026_H
