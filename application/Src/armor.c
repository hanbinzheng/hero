#include "armor.h"
#include "dbus.h"
#include "dji_motor.h"
#include "dm_motor.h"

#define ARMOR_GEAR_RATIO (37.0f / 117.0f)
#define ARMOR_SENSITIVITY (0.8f)

static float vel_friction_base[6] = {-100.0f, 100.0f, 100.0f, -100.0f, 100.0f, 100.0f};
static float vel_stop[6] = {0};
static uint64_t armor_debug = 0;
static float curr_total = 0;

static inline void update_vel(float vel[6], float vel_scale, float limit) {
	for (int i = 0; i < 6; i++) {
		if (vel[i] >= limit || vel[i] <= -limit) {
			return;
		}

		if (vel[i] < 0) {
			vel[i] = vel[i] - vel_scale;
		} else if (vel[i] > 0) {
			vel[i] = vel[i] + vel_scale;
		}
	}
}

static inline void reset_vel_base(void) {
	vel_friction_base[0] = -100.0f;
	vel_friction_base[1] = 100.0f;
	vel_friction_base[2] = 100.0f;
	vel_friction_base[3] = -100.0f;
	vel_friction_base[4] = 100.0f;
	vel_friction_base[5] = 100.0f;
}

static inline float get_friction_cur(void) {
	return dji3508_10.cur + dji3508_9.cur + dji3508_8.cur + dji3508_7.cur + dji3508_6.cur + dji3508_5.cur;
}

/* friction: 125hz, armor booster: 25hz */
void armor_task(void)
{
	armor_debug++; /* only for debug usage */

	/* state machine, with frequency 125 hz */
	static uint16_t armor_count = 0;
	armor_count = (armor_count + 1) % 125;

	/* control info */
	static float pos_armor = 0.0f;
	static volatile float vel_armor = DM4310_VEL_MAX;

	/* safe mode */
	if (dbus_data.sw1 == SW_UP) {
		dm4310_send_command(0.0f, 0.0f); /* vel = 0, remain still */
		dji3508_set_armor_vel(vel_stop);
		return;
	}

	/* control loop for armor booster, 25hz */
	if (armor_count % 5 == 0) {
		pos_armor += dbus_data.rs_y * ARMOR_SENSITIVITY;
		dm4310_send_command(pos_armor, vel_armor);
	} else if (armor_count == 124) { /* a loop: enable it */
		dm4310_enable();
	}

	/* armor friction control */
	if (dbus_data.sw2 == SW_UP) {
		dji3508_set_armor_vel(vel_friction_base);

		/* slowly update velocity */
		if (armor_count % 25 == 0 ) {
			update_vel(vel_friction_base, 100.0f, 500.0f);
		}
		curr_total = get_friction_cur();
	} else {
		dji3508_set_armor_vel(vel_stop);
		reset_vel_base();
	}
}
