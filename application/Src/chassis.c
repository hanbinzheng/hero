#include "chassis.h"
#include "arm_math.h"
#include "dbus.h"
#include "dji_motor.h"
#include "kinematics.h"
#include <math.h>

#define RADIUS (0.059f)
#define SENSITIVITY (2.0f)
#define V_ROTATE (2.0f)
#define V_THRESHOLD (2.0f)
#define V_UPDATE_MIN (0.05f)
#define ABS(x) ((x > 0) ? (x) : (-x))

static int const wheel_direction[4] = {1, -1, -1, 1};

static void v_limit(float v[4])
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
			vel_cmd[i] = vel[i] * wheel_direction[i] / RADIUS;
		} else {
			pos_cmd[i] = set_in_range(pos[i] + PI);
			vel_cmd[i] = -vel[i] * wheel_direction[i] / RADIUS;
		}
	}
}

void chassis_task()
{
	static float vx_cmd = 0, vy_cmd = 0, v_rotate = 0;
	static float pos_raw[4], pos_cmd[4], vel_raw[4], vel_cmd[4];

	if (dbus_data.sw1 == SW_UP) {
		dji6020_set_pos(PI / 4, -PI / 4, PI / 4, -PI / 4);
		dji3508_set_chassis_vel(0, 0, 0, 0);
		return;
	}

	/* get command data */
	vx_cmd = dbus_data.ls_x * SENSITIVITY;
	vy_cmd = -dbus_data.ls_y * SENSITIVITY;
	v_rotate = -dbus_data.rs_y * V_ROTATE;

	/* interpret data into motor command */
	float v_cmd[3] = {vx_cmd, vy_cmd, v_rotate};
	kinematics_swerve(v_cmd, 0, pos_raw, vel_raw);
	get_cmd(pos_raw, vel_raw, pos_cmd, vel_cmd); /* shortest path */
	v_limit(vel_cmd);

	/* send command */
	dji3508_set_chassis_vel(vel_cmd[0], vel_cmd[1], vel_cmd[2], vel_cmd[3]);
	dji6020_set_pos(pos_cmd[0], pos_cmd[1], pos_cmd[2], pos_cmd[3]);
}
