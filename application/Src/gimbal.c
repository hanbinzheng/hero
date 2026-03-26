#include "gimbal.h"
#include "dbus.h"
#include "mi_motor.h"

#define HORIZONTAL_POS (0.0f)
#define PITCH_DEPRESSION (-1.2981f)
#define PITCH_ELEVATION (-0.3179f)
#define PITCH_TOTAL_SCALE (0.98023f) /* evaluation - depression */
#define PITCH_SENSITIVITY (0.005f)
#define PITCH_BALANCE (-0.50f)

static inline void limit(float *value, float min, float max)
{
	if (*value >= max) {
		*value = max;
	} else if (*value <= min) {
		*value = min;
	}
}

void gimbal_task(void)
{
	static volatile float vel_pitch = 1.0f; /* radius */
	static float vel_yaw = 0.0f;
	static float pos_pitch = -0.808015f;
	static float trq_pitch = 0.0f;

	if (dbus_data.sw1 == SW_UP) {
		mi_send_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		return;
	}

	pos_pitch = -dbus_data.rs_x * PITCH_TOTAL_SCALE + (PITCH_DEPRESSION + PITCH_ELEVATION) / 2;
	limit(&pos_pitch, PITCH_DEPRESSION, PITCH_ELEVATION);
	mi_set_pos(pos_pitch);
}
