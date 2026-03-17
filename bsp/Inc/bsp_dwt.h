/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @author  modified by NeoZng
 * @author  modified by Zheng Hanbin
 * @date    2026/03/17
 * @brief
 ******************************************************************************
 */
#ifndef __BSP_DWT_H__
#define __BSP_DWT_H__

#include <stdint.h>
#include "main.h"

struct dwt_time
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
};

/**
 * @brief used to calculate code segment execution time in seconds
 * 
 * @param dt: float
 * @param code: code segmentation to execute
 */
#define TIME_ELAPSE(dt, code)                   \
    do {                                        \
        float tstart = dwt_get_timeline_s();    \
        code;                                   \
        dt = dwt_get_timeline_s() - tstart;     \
    } while (0)                                 \

/**
 * @brief Initialize DWT, input parameter is CPU frequency, unit MHz
 *
 * @param CPU_Freq_mHz C board is 168MHz, A board is 180MHz
 */
void dwt_init(uint32_t cpu_freq_mhz);

/**
 * @brief Get the time interval between two calls, unit is seconds
 *
 * @attention assume that within one overflow
 * @param cnt_last Timestamp of the last call
 * @return float Time interval, unit is seconds
 */
float dwt_get_delta_t(uint32_t *cnt_last);

/**
 * @brief Get the time interval between two calls, unit is seconds, high precision
 *
 * @param cnt_last Timestamp of the last call
 * @return double Time interval, unit is seconds
 */
double dwt_get_delta_t_64(uint32_t *cnt_last);

/**
 * @brief Get current time, unit is seconds, time since initialization
 *
 * @return float Timeline
 */
float dwt_get_timeline_s(void);

/**
 * @brief Get current time, unit is milliseconds, time since initialization
 *
 * @return float
 */
float dwt_get_timeline_ms(void);

/**
 * @brief Get current time, unit is microseconds, time since initialization
 *
 * @return uint64_t
 */
uint64_t dwt_get_timeline_us(void);

/**
 * @brief DWT delay function, unit is seconds
 * @attention not affected by whether interrupts are enabled
 * @note Do not use HAL_Delay() function between __disable_irq() and __enable_irq(), use this function instead
 *
 * @param Delay Delay time, unit is seconds
 */
void dwt_delay(float delay);

/**
 * @brief DWT update timeline function, will be called by the three timeline functions
 * @attention If timeline functions are not called for a long time, this function needs to be called manually to update the timeline, otherwise CYCCNT overflow will cause inaccurate timing and timeline
 */
void dwt_systime_update(void);

#endif // __BSP_DWT_H__
