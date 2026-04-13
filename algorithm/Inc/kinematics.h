#ifndef KINEMATICS_H_
#define KINEMATICS_H_

/*
 * vel: velocity under gimbal frame
 *     (vx_g: m/s, vy_g: m/s, v_rotate: m/s)
 *     v_rotate = w * length, calculated by user itself
 * yaw_diff: gimbal frame yaw - chassis frame yaw
 * tgt_vx: x coordinate velocity of 0/1/2/3 under chassis frame
 * tgt_vy: xxx
 *
 *           ^ x
 *           |
 *     2-----------1
 * y   |     |     |
 * <---|-----|     |
 *     |           |
 *     3-----------4
 */

void kinematics_swerve(float vel[3], float yaw_diff, float tgt_pos[4], float tgt_vel[4]);

#endif /* KINEMATICS_H_ */
