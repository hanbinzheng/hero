#include "kinematics.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>

#define SQRT_2 1.4142135623730951
#define SQRT_2_OVER_2 0.7071067811865475
#define NORM_SQUARE(x, y) ((x) * (x) + (y) * (y))

/*
** vel: velocity under gimbal frame
**     (vx_g: m/s, vy_g: m/s, v_rotate: m/s)
**     v_rotate = w * length, calculated by user itself
** yaw_diff: gimbal frame yaw - chassis frame yaw
** tgt_vx: x coordinate velocity of 0/1/2/3 under chassis frame
** tgt_vy: xxx
**
**           ^ x
**           |
**     2-----------1
** y   |     |     |
** <---|-----|     |
**     |           |
**     3-----------4
*/
void kinematics_swerve(float vel[3], float yaw_diff, float tgt_vx[4], float tgt_vy[4])
{
	/* get velocities under the gimbal frame */
	float vx_g = vel[0];
	float vy_g = vel[1];
	float v_rotate = vel[2];

	float32_t cos_val = arm_cos_f32(yaw_diff);
	float32_t sin_val = arm_sin_f32(yaw_diff);

	/* get velocities under the chassis frame
	 *
	 * [ vx_c ]  = [ cos -sin ] [ vx_g ]
	 * [ vy_c ]  = [ sin  cos ] [ vy_g ]
	 */
	float vx_c = cos_val * vx_g - sin_val * vy_g;
	float vy_c = cos_val * vy_g + sin_val * vx_g;

	/* get target velocity under chassis frame
	 *
	 * v1 = vc + v_rotate / SQRT_2 * (1, 1)
	 * v2 = vc + v_rotate / SQRT_2 * (-1, 1)
	 * v3 = vc + v_rotate / SQRT_2 * (-1, -1)
	 * v4 = vc + v_rotate / SQRT_2 * (1, -1)
	 *
	 */
	tgt_vx[0] = vx_c + v_rotate * SQRT_2_OVER_2;
	tgt_vx[1] = vx_c - v_rotate * SQRT_2_OVER_2;
	tgt_vx[2] = vx_c - v_rotate * SQRT_2_OVER_2;
	tgt_vx[3] = vx_c + v_rotate * SQRT_2_OVER_2;

	tgt_vy[0] = vy_c + v_rotate * SQRT_2_OVER_2;
	tgt_vy[1] = vy_c + v_rotate * SQRT_2_OVER_2;
	tgt_vy[2] = vy_c - v_rotate * SQRT_2_OVER_2;
	tgt_vy[3] = vy_c - v_rotate * SQRT_2_OVER_2;
}
