#include "chassis.h"
#include "arm_math.h"
#include "dbus.h"
#include "dji_motor.h"
#include "kinematics.h"
#include <math.h>

#define RADIUS (0.059f)
static float vx_cmd = 0;
static float vy_cmd = 0;
static float v_rotate = 0;
static float tgt_vx[4];
static float tgt_vy[4];
static float pos_cmd[4];
static float vel_cmd[4];
static int const wheel_direction[4] = {1, -1, -1, 1};

#define SENSITIVITY (2.0f)
#define V_THRESHOLD (2.0f)
#define ABS(x) ((x > 0) ? (x) : (-x))

static void limit(float v[4])
{
	for (int i = 0; i < 4; i++) {
		if (v[i] > V_THRESHOLD / RADIUS) {
			v[i] = V_THRESHOLD / RADIUS;
		} else if (v[i] < -V_THRESHOLD / RADIUS) {
			v[i] = -V_THRESHOLD / RADIUS;
		}
	}
}

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

static void get_cmd(float vx[4], float vy[4], float pos[4], float vel[4])
{
	/* get steer position measurement */
	static float pos_measure[4], pos_raw[4], vel_raw[4];
	pos_measure[0] = update_pos(&dji6020_1, GM6020_ANGLE_OFFSET_1);
	pos_measure[1] = update_pos(&dji6020_2, GM6020_ANGLE_OFFSET_2);
	pos_measure[2] = update_pos(&dji6020_3, GM6020_ANGLE_OFFSET_3);
	pos_measure[3] = update_pos(&dji6020_4, GM6020_ANGLE_OFFSET_4);

	/* calculate raw position and velocity */
	for (int i = 0; i < 4; i++) {
		arm_sqrt_f32(vx[i] * vx[i] + vy[i] * vy[i], &vel_raw[i]);
		if (vel_raw[i] < 0.05f) {
			/* for joltting case */
			pos[i] = pos_measure[i];
			vel[i] = 0;
			continue;
		}

		pos_raw[i] = atan2f(vy[i], vx[i]);
	}

	/* optimize command */
	for (int i = 0; i < 4; i++) {
		float diff = set_in_range(pos_raw[i] - pos_measure[i]);
		if (ABS(diff) <= PI / 2) {
			pos[i] = pos_raw[i];
			vel[i] = vel_raw[i] * wheel_direction[i] / RADIUS;
		} else {
			pos[i] = set_in_range(pos_raw[i] + PI);
			vel[i] = -vel_raw[i] * wheel_direction[i] / RADIUS;
		}
	}
}

void chassis_task()
{
	if (dbus_data.sw1 == SW_UP) {
		dji6020_set_pos(PI / 4, -PI / 4, PI / 4, -PI / 4);
		dji3508_set_chassis_vel(0, 0, 0, 0);
		return;
	}

	/* get command data */
	vx_cmd = dbus_data.ls_x * SENSITIVITY;
	vy_cmd = -dbus_data.ls_y * SENSITIVITY;
	v_rotate = dbus_data.rs_x * 2;

	/* interpret data into motor command */
	float v_cmd[3] = {vx_cmd, vy_cmd, v_rotate};
	kinematics_swerve(v_cmd, 0, tgt_vx, tgt_vy);
	get_cmd(tgt_vx, tgt_vy, pos_cmd, vel_cmd);
	limit(vel_cmd);

	/* send command */
	dji3508_set_chassis_vel(vel_cmd[0], vel_cmd[1], vel_cmd[2], vel_cmd[3]);
	dji6020_set_pos(pos_cmd[0], pos_cmd[1], pos_cmd[2], pos_cmd[3]);
}
