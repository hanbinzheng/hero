#include "armor.h"
#include "dbus.h"
#include "vt03.h"
#include "dji_motor.h"
#include "dm_motor.h"

#define ARMOR_GEAR_RATIO (37.0f / 117.0f)
#define ARMOR_SENSITIVITY (0.5f)
#define NUM_BULLET_A_ROUND 9

static float vel_friction_base[6] = {-50.0f, 50.0f, 50.0f, -100.0f, 100.0f, 100.0f};
static float vel_stop[6] = {0};
static uint64_t armor_debug = 0;
static float curr_total = 0;

static inline void update_vel(float vel[6], float vel_scale, 
	float limit_inner, float limit_outer) {
		for (int i = 0; i < 3; i++) {
			if (vel[i] < 0 && (vel[i] - vel_scale > - limit_inner)) {
				vel[i] -= vel_scale;
			} else if (vel[i] > 0 && (vel[i] + vel_scale < limit_inner)) {
				vel[i] += vel_scale;
			}
		}

		for (int i = 3; i < 6; i++) {
			if (vel[i] < 0 && vel[i] - vel_scale > - limit_outer) {
				vel[i] -= vel_scale;
			} else if (vel[i] > 0 && vel[i] + vel_scale < limit_outer) {
				vel[i] += vel_scale;
			}
		}
}

static inline void reset_vel_base(void) {
	vel_friction_base[0] = -50.0f;
	vel_friction_base[1] = 50.0f;
	vel_friction_base[2] = 50.0f;
	vel_friction_base[3] = -50.0f;
	vel_friction_base[4] = 50.0f;
	vel_friction_base[5] = 50.0f;
}

static inline float get_friction_cur(void) {
	return dji3508_10.cur + dji3508_9.cur + dji3508_8.cur + dji3508_7.cur + dji3508_6.cur + dji3508_5.cur;
}

/* friction: 125hz, armor booster: 25hz */
void armor_task(void)
{
	armor_debug++; /* only for debug usage */
	static uint8_t armor_booster_reverse = 0; /* 1: reversed, 0: not reversed */
	static uint8_t armor_booster_moved = 0; /* 1: moved, 0: not moved */

	/* control info */
	static float pos_armor = 0.0f;
	static volatile float vel_armor = DM4310_VEL_MAX;

	/* state machine, with frequency 125 hz */
	static uint16_t armor_count = 0;

	// if (armor_count == 0) { /* safety mechanism */
	// 	pos_armor = dm4310.pos;
	// }

	/* safe mode */
	if (vt03_data.mode_sw == MODE_C) {
		dm4310_send_command(0.0f, 0.0f); /* vel = 0, remain still */
		dji3508_set_armor_vel(vel_stop);
		return;
	}

	/* control loop for armor booster, 25hz */
	if (armor_count % 5 == 0) {
		if (vt03_data.wheel < 0 && armor_booster_moved == 0) {
			pos_armor -= (2 * PI / NUM_BULLET_A_ROUND) / ARMOR_GEAR_RATIO;
			armor_booster_reverse = 0;
			armor_booster_moved = 1;
			dm4310_send_command(pos_armor, vel_armor);
		} else if (vt03_data.wheel > 0 && armor_booster_reverse == 0) {
			/* mechanics: can only reverse half bullet */
			pos_armor += 0.5 * (2 * PI / NUM_BULLET_A_ROUND) / ARMOR_GEAR_RATIO;
			armor_booster_reverse = 1;
			dm4310_send_command(pos_armor, vel_armor);
		}
	} else if (armor_count == 83 || armor_count == 166) {
		armor_booster_moved = 0;
	} else if (armor_count == 249) { /* a loop: enable it */
		dm4310_enable();
	}

	/* armor friction control */
	if (vt03_data.trigger == 1) {
		dji3508_set_armor_vel(vel_friction_base);
		/* slowly update velocity */
		if (armor_count % 10 == 0 ) {
			update_vel(vel_friction_base, 20.0f, 300.0f, 460.0f);
		}
		curr_total = get_friction_cur();
	} else {
		dji3508_set_armor_vel(vel_stop);
		reset_vel_base();
	}

	/* update counter */
	armor_count = (armor_count + 1) % 250; /* 3 bullet in 2 s */
}
