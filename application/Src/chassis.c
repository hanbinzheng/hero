#include "chassis.h"
#include "arm_math.h"
#include "dbus.h"
#include "vt03.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "kinematics.h"
#include <math.h>

#define WHEEL_RADIUS (0.059f)

#define V_UPDATE_MIN (0.05f)
#define ABS(x) ((x > 0) ? (x) : (-x))

/* parameters for spinning mode */
#define R_CHASSIS (0.2475f)
#define OMEGA (3 * PI)

#define CHASSIS_INCREMENT (0.016f) /* 125hz */
#define CHASSIS_REDUCTION_SCALE (0.90f)

#define CHASSIS_VEL_LEVEL (2.0f)
#define CHASSIS_V_ROTATE_LEVEL (2.4f)

static int const wheel_direction[4] = {1, -1, -1, 1};

static float set_in_range(float pos)
{
	while (pos > PI) {
		pos -= 2 * PI;
	}
	while (pos < -PI) {
		pos += 2 * PI;
	}
	return pos;
}

void get_cmd(float pos[4], float vel[4], float pos_cmd[4], float vel_cmd[4])
{
	/* get measurement of GM6020 */
	static float pos_measure[4];
	pos_measure[0] = dji_get_pos(&dji6020_1, GM6020_ANGLE_OFFSET_1);
	pos_measure[1] = dji_get_pos(&dji6020_2, GM6020_ANGLE_OFFSET_2);
	pos_measure[2] = dji_get_pos(&dji6020_3, GM6020_ANGLE_OFFSET_3);
	pos_measure[3] = dji_get_pos(&dji6020_4, GM6020_ANGLE_OFFSET_4);

	for (int i = 0; i < 4; i++) {
		if (vel[i] < V_UPDATE_MIN) { /* avoid jolting */
			vel_cmd[i] = 0;
			pos_cmd[i] = pos_measure[i];
			continue;
		}

		float diff = set_in_range(pos[i] - pos_measure[i]);
		if (ABS(diff) <= PI / 2) {
			pos_cmd[i] = pos[i];
			vel_cmd[i] = vel[i] * wheel_direction[i] / WHEEL_RADIUS;
		} else {
			pos_cmd[i] = set_in_range(pos[i] + PI);
			vel_cmd[i] = -vel[i] * wheel_direction[i] / WHEEL_RADIUS;
		}

		/* adjust velocity to avoid skidding */
		float error = pos_cmd[i] - pos_measure[i];
		vel_cmd[i] = vel_cmd[i] * arm_cos_f32(error);
	}
}

static inline float clamp(float x, float min, float max) {
	if (x < min) {
		return min;
	} else if (x > max) {
		return max;
	} else {
		return x;
	}
}

static float slope_x(int x){
	static float ret = 0;
	switch(x){
		case 1: ret += CHASSIS_INCREMENT; break;
		case -1: ret -= CHASSIS_INCREMENT; break;
      	case 0: ret *= CHASSIS_REDUCTION_SCALE; break;
   	}
	ret = clamp(ret, -1.0f, 1.0f);
	return ret;
}

static float slope_y(int y){
	static float ret = 0;
	switch(y){
		case 1: ret += CHASSIS_INCREMENT; break;
		case -1: ret -= CHASSIS_INCREMENT; break;
      	case 0: ret *= CHASSIS_REDUCTION_SCALE; break;
   	}
	ret = clamp(ret, -1.0f, 1.0f);
	return ret;
}

uint64_t chassis_debug = 0;
void chassis_task()
{
	chassis_debug++; /* only for debug usage */
	static float x = 0, y = 0, z = 0; /* x, y and rotate */
	static float vx_cmd = 0, vy_cmd = 0, v_rotate = 0, yaw_diff = 0;
	static float pos_raw[4], pos_cmd[4], vel_raw[4], vel_cmd[4];

	if (vt03_data.mode_sw == MODE_C) {
		static float safe_vel[4] = {0, 0, 0, 0};
		static float safe_pos[4] = {-PI / 4, PI / 4, -PI / 4, PI / 4};
		dji6020_set_pos(safe_pos);
		dji3508_set_chassis_vel(safe_vel);
		return;
	} else if (vt03_data.mode_sw == MODE_N) {
		z = 0;
		yaw_diff = dm6006_get_pos();
	} else { /* MODE_S */
		if (vt03_data.keyboard.bit.q || vt03_data.l_fn) {
			z = (z + CHASSIS_INCREMENT >= 1.0f) ? 1.0f : z + CHASSIS_INCREMENT;
		} else if (vt03_data.keyboard.bit.e || vt03_data.r_fn) {
			z = (z - CHASSIS_INCREMENT <= -1.0f) ? -1.0f : z - CHASSIS_INCREMENT;
		} else if (vt03_data.keyboard.bit.r || vt03_data.pause) {
			z = z * CHASSIS_REDUCTION_SCALE;
		}
		yaw_diff = dm6006_get_pos();
	}

	/* get command of x direction and y direction */
	x = vt03_data.ls_x + slope_x(vt03_data.keyboard.bit.w - vt03_data.keyboard.bit.s);
	y = vt03_data.ls_y + slope_y(vt03_data.keyboard.bit.a - vt03_data.keyboard.bit.d);

	/* get command data */
	vx_cmd = x * CHASSIS_VEL_LEVEL;
	vy_cmd = y * CHASSIS_VEL_LEVEL;
	v_rotate = z * CHASSIS_V_ROTATE_LEVEL;

	/* interpret data into motor command */
	float v_cmd[3] = {vx_cmd, vy_cmd, v_rotate};
	kinematics_swerve(v_cmd, yaw_diff, pos_raw, vel_raw);
	get_cmd(pos_raw, vel_raw, pos_cmd, vel_cmd); /* shortest path */

	/* send command */
	dji3508_set_chassis_vel(vel_cmd);
	dji6020_set_pos(pos_cmd);
}
