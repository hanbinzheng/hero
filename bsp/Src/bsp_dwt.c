/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @author  modified by Neo with annotation
 * @author  modified by Zheng Hanbin
 * @date    2026/03/17
 * @brief
 ******************************************************************************
 */

#include "bsp_dwt.h"

static struct dwt_time systime;
static uint32_t cpu_freq_hz, cpu_freq_hz_ms, cpu_freq_hz_us;
static uint32_t cyccnt_rountcount;
static uint32_t cyccnt_last;
static uint64_t cyccnt64;

/**
 * @brief private function used to check if DWT CYCCNT register overflows and update
 * CYCCNT_RountCount
 * @attention this function assumes the time interval between two calls does not
 * exceed one overflow
 *
 * @todo Better solution: set up a separate task for DWT time update?
 *        However, the original intention of using dwt is to ensure timing is not
 * affected by factors such as interrupts/tasks, so this implementation still has its
 * significance
 *
 */
static void dwt_cnt_update(void)
{
	static volatile uint8_t bit_locker = 0;
	if (!bit_locker) {
		bit_locker = 1;
		volatile uint32_t cnt_now = DWT->CYCCNT;
		if (cnt_now < cyccnt_last)
			cyccnt_rountcount++;

		cyccnt_last = DWT->CYCCNT;
		bit_locker = 0;
	}
}

void dwt_init(uint32_t cpu_freq_mhz)
{
	// enable dwt periphery
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	// clear dwt CYCCNT register
	DWT->CYCCNT = (uint32_t)0u;

	// enable Cortex-M DWT CYCCNT register
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	cpu_freq_hz = cpu_freq_mhz * 1000000;
	cpu_freq_hz_ms = cpu_freq_hz / 1000;
	cpu_freq_hz_us = cpu_freq_hz / 1000000;
	cyccnt_rountcount = 0;

	dwt_cnt_update();
}

float dwt_get_delta_t(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(cpu_freq_hz));
	*cnt_last = cnt_now;

	dwt_cnt_update();

	return dt;
}

double dwt_get_delta_t_64(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(cpu_freq_hz));
	*cnt_last = cnt_now;

	dwt_cnt_update();

	return dt;
}

void dwt_systime_update(void)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	static uint64_t cnt_temp1, cnt_temp2, cnt_temp3;

	dwt_cnt_update();

	cyccnt64 = (uint64_t)cyccnt_rountcount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
	cnt_temp1 = cyccnt64 / cpu_freq_hz;
	cnt_temp2 = cyccnt64 - cnt_temp1 * cpu_freq_hz;
	systime.s = cnt_temp1;
	systime.ms = cnt_temp2 / cpu_freq_hz_ms;
	cnt_temp3 = cnt_temp2 - systime.ms * cpu_freq_hz_ms;
	systime.us = cnt_temp3 / cpu_freq_hz_us;
}

float dwt_get_timeline_s(void)
{
	dwt_systime_update();

	float dwt_timeline_f32 = systime.s + systime.ms * 0.001f + systime.us * 0.000001f;

	return dwt_timeline_f32;
}

float dwt_get_timeline_ms(void)
{
	dwt_systime_update();

	float dwt_timeline_f32 = systime.s * 1000 + systime.ms + systime.us * 0.001f;

	return dwt_timeline_f32;
}

uint64_t dwt_get_timeline_us(void)
{
	dwt_systime_update();

	uint64_t dwt_timeline = systime.s * 1000000 + systime.ms * 1000 + systime.us;

	return dwt_timeline;
}

void dwt_delay_us(uint32_t delay_us)
{
	uint32_t tickstart = DWT->CYCCNT;
	uint32_t ticks = delay_us * (cpu_freq_hz / 1000000);

	while ((DWT->CYCCNT - tickstart) < ticks)
		;
}

void dwt_delay_ms(uint32_t delay_ms)
{
	uint32_t tickstart = DWT->CYCCNT;
	uint32_t ticks = delay_ms * (cpu_freq_hz / 1000);

	while ((DWT->CYCCNT - tickstart) < ticks)
		;
}

void dwt_delay_s(float delay_s)
{
	uint32_t tickstart = DWT->CYCCNT;
	float wait = delay_s;

	while ((DWT->CYCCNT - tickstart) < wait * (float)cpu_freq_hz)
		;
}
