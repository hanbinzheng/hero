#include "ammo.h"
#include "dbus.h"
#include "vt03.h"
#include "referee.h"
#include "dji_motor.h"
#include "dm_motor.h"

#ifndef PI
#define PI (3.141592653589793238f)
#endif

#define AMMO_BOOSTER_GEAR_RATIO (37.0f / 117.0f)
#define NUM_BULLET_A_ROUND 9
#define ANGLE_OF_A_BULLET (2 * PI / NUM_BULLET_A_ROUND / AMMO_BOOSTER_GEAR_RATIO)
#define TRIGGER_UPDATE_SCALE (ANGLE_OF_A_BULLET / 125.0f * 5.0f)

#define VEL_FRICTION_INNER_MAX (400.0f)
#define VEL_FRICTION_OUTER_MAX (400.0f)
#define FRICTION_REDUCTION_SCALE (0.90f)
#define FRICTION_INCREMENT_SCALE (5.0f)

uint64_t ammo_debug = 0; /* incrementing global variables, 125/s */
static uint8_t friction_on = 0;
static uint8_t friction_state = 0; /* 0: off, 1: on, change by pause */

static const float friction_direction[6] = {-1.0, 1.0, 1.0, -1.0, 1.0, 1.0};
static float vel_friction_stop[6] = {0};
static float vel_friction[6] = {0};

static float limit_by_direction(float threshold, float real, float increment)
{
    float new_value = real + increment;
    if (increment >= 0) {
        return (new_value > threshold) ? threshold : new_value;
    } else {
        return (new_value < threshold) ? threshold : new_value;
    }
}

static float slope_trigger(int value) {
    static float ret = 0.0f;
    if (value == 1){
        ret -= TRIGGER_UPDATE_SCALE;
    }
    return ret;
}

static void update_friction(uint8_t state) {
    if (state == 0) {
        for (int i = 0; i < 6; i++) {
            vel_friction[i] *= FRICTION_REDUCTION_SCALE;
        }
    } 
    else if (state == 1) {
        /* inner */
        for (int i = 0; i < 3; i++) {
            float dir = friction_direction[i];
            float bound = VEL_FRICTION_INNER_MAX * dir;
            float delta = FRICTION_INCREMENT_SCALE * dir;
            vel_friction[i] = limit_by_direction(bound, vel_friction[i], delta);
        }
        /* outer */
        for (int i = 3; i < 6; i++) {
            float dir = friction_direction[i];
            float bound = VEL_FRICTION_OUTER_MAX * dir;
            float delta = FRICTION_INCREMENT_SCALE * dir;
            vel_friction[i] = limit_by_direction(bound, vel_friction[i], delta);
        }
    }
}

void ammo_task(void)
{
	static uint16_t ammo_count = 0;
	ammo_debug++;
    static float pos_trigger = 0.0f;

	/* safe mode, for safety */
	if (vt03_data.mode_sw == MODE_C) {
		dji3508_set_ammo_vel(vel_friction_stop); /* set vel to 0 */
		dm4310_send_command(dm4310.pos, 0.0f); /* current position, stay still */
		ammo_count = (ammo_count + 1) % 125;
		return;
	}

	/* change the state of friction: shift or ctrl */
	if (vt03_data.keyboard.bit.ctrl == 1 || vt03_data.trigger == 0) {
		friction_state = 0; /* ctrl: off */
	} else if (vt03_data.keyboard.bit.shift == 1 || vt03_data.trigger == 1) {
		friction_state = 1; /* shift: on */
	}
	update_friction(friction_state);
    	dji3508_set_ammo_vel(vel_friction);

	/* enable dm4310 motor at a const frequenccy */
	if (ammo_count == 124) {
		dm4310_enable();
	}

    /* control logic for ammo trigger/booster */
    if (referee_info.power_heat_data.shooter_42mm_barrel_heat >= referee_info.robot_status.shooter_barrel_heat_limit) {
        pos_trigger = dm4310.pos; /* overheated */
    } else {
		pos_trigger = slope_trigger(vt03_data.mouse_left || (vt03_data.wheel < 0)); /* update trigger position */
	}
    	dm4310_send_command(pos_trigger, DM4310_VEL_MAX);

	/* update state machine counter */
	ammo_count = (ammo_count + 1) % 125;
}
