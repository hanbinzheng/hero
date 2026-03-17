#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "arm_math.h"
#include <stdint.h>

struct quaternion {
	float q_w; /* real part */
	float q_x; /* imaginary i */
	float q_y; /* imaginary j */
	float q_z; /* imaginary k */
};

/* set and copy */
void quat_set(struct quaternion *q, float w, float x, float y, float z);
void quat_identity(struct quaternion *q);
void quat_copy(struct quaternion *dest, struct quaternion *src);

/* basic algebra */
void quat_add(struct quaternion *a, struct quaternion *b, struct quaternion *result);
void quat_subtract(struct quaternion *a, struct quaternion *b,
		   struct quaternion *result);
void quat_multiply(struct quaternion *a, struct quaternion *b,
		   struct quaternion *result);
void quat_scale(struct quaternion *q, float scalar, struct quaternion *result);

/* vector related algebra */
float quat_norm(struct quaternion *q);
void quat_normalize(struct quaternion *q);
void quat_conjugate(struct quaternion *q, struct quaternion *result);
void quat_inverse(struct quaternion *q,
		  struct quaternion *result); /* conjugation + norm */

/* quaternion derivative operation */
void quat_derivative(struct quaternion *q, float w[3], struct quaternion *result);

/* quaternion and vector rotation */
void quat_from_axis_angle(float axis[3], float angle, struct quaternion *q);
void quat_to_axis_angle(struct quaternion *q, float axis[3], float *angle);
void quat_rotate_vector(struct quaternion *q, float vec[3], float result[3]);

/* quaternion and euler angles */
/* euler[0]: yaw, euler[1]: pitch, euler[2]: roll */
void quat_from_euler(float euler[3], struct quaternion *q);
void quat_to_euler(struct quaternion *q, float euler[3]);

#endif /* QUATERNION_H_ */
