#include "gimbal.h"
#include "dbus.h"
#include "vt03.h"
#include "imu.h"
#include "dm_motor.h"
#include "mi_motor.h"

#define HORIZONTAL_POS (0.0f)
#define PITCH_DEPRESSION (-1.2981f)
#define PITCH_ELEVATION (-0.3179f)
#define PITCH_TOTAL_SCALE (0.98023f) /* evaluation - depression */
#define PITCH_SENSITIVITY (0.005f)
#define PITCH_BALANCE (-0.50f)
#define PITCH_SENSITIVITY (0.003f)
#define YAW_SENSITIVITY (0.0075f)

static uint64_t gimbal_debug = 0;
static uint8_t dm6006_stop[8] = {0};

static inline void limit(float *value, float min, float max)
{
	if (*value >= max) {
		*value = max;
	} else if (*value <= min) {
		*value = min;
	}
}

static inline void correct_yaw_pos_ref(float *value) {
	while (*value >= PI) {
		*value -= 2 * PI;
	}

	while (*value <= -PI) {
		*value += 2 * PI;
	}
}

void gimbal_task(void)
{
	/* basic control info */
	static volatile float vel_pitch = 1.0f; /* in radius, for debug */
	static float vel_yaw = 0.0f;
	static float pos_pitch = -0.808015f;
	static float pos_yaw = 0.0f;
	static float trq_pitch = 0.0f;

	if (gimbal_debug == 0) {
		pos_yaw = dm6006_get_pos(dm6006.raw_pos);
		pos_pitch = mi_motor.pos;
	}
	gimbal_debug++; /* only for debug usage */
	imu_update();

	/* 1000hz control frequency, the state machine */
	static uint16_t gimbal_count = 0;
	gimbal_count = (gimbal_count + 1) % 1000;

	/* safe mode */
	if (vt03_data.mode_sw == MODE_S) {
		mi_send_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f); /* velocity = 0: keep stable */
		can_transmit(&hfdcan1, 0x3FE, CAN_ID_STD, dm6006_stop);

		/* for safety */
		pos_yaw = dm6006_get_pos(dm6006.raw_pos);
		vel_yaw = 0;
		vel_pitch = 0;
		pos_pitch = mi_motor.pos;
		trq_pitch = 0;
		return;
	}

	/* yaw control */
	pos_yaw += vt03_data.rs_y * YAW_SENSITIVITY;
	correct_yaw_pos_ref(&pos_yaw);
	dm6006_set_pos(pos_yaw);
	// vel_yaw = dbus_data.ls_x * 12;
	// dm6006_set_vel(vel_yaw);

	/* control for pitch motor */
	if (gimbal_count == 0) { /* enable the mi motor every second in case of reboot */
		mi_motor_enable();
	} else {
		/* pitch control */
		pos_pitch += -vt03_data.rs_x * PITCH_SENSITIVITY;
		limit(&pos_pitch, PITCH_DEPRESSION, PITCH_ELEVATION);
		mi_set_pos(pos_pitch);
	}
}
