#include "quaternion.h"
#include <math.h>

#define QUAT_EPSILON (1.0e-6f) /* threshold */
#define QUAT_PI (3.14159265358979323846f)

static inline float quat_vector_norm_squared(float *vec, uint32_t len)
{
	float result;
	arm_dot_prod_f32(vec, vec, len, &result);
	return result;
}

static inline float quat_vector_norm(float *vec, uint32_t len)
{
	float norm_square, norm;
	arm_dot_prod_f32(vec, vec, len, &norm_square);
	arm_sqrt_f32(norm_square, &norm);
	return norm;
}

void quat_set(struct quaternion *q, float w, float x, float y, float z)
{
	q->q_w = w;
	q->q_x = x;
	q->q_y = y;
	q->q_z = z;
}

void quat_identity(struct quaternion *q)
{
	q->q_w = 1.0f;
	q->q_x = 0.0f;
	q->q_y = 0.0f;
	q->q_z = 0.0f;
}

void quat_copy(struct quaternion *dest, struct quaternion *src)
{
	dest->q_w = src->q_w;
	dest->q_x = src->q_x;
	dest->q_y = src->q_y;
	dest->q_z = src->q_z;
}

void quat_add(struct quaternion *a, struct quaternion *b, struct quaternion *result)
{
	result->q_w = a->q_w + b->q_w;
	result->q_x = a->q_x + b->q_x;
	result->q_y = a->q_y + b->q_y;
	result->q_z = a->q_z + b->q_z;
}

void quat_subtract(struct quaternion *a, struct quaternion *b, struct quaternion *result)
{
	result->q_w = a->q_w - b->q_w;
	result->q_x = a->q_x - b->q_x;
	result->q_y = a->q_y - b->q_y;
	result->q_z = a->q_z - b->q_z;
}

void quat_multiply(struct quaternion *a, struct quaternion *b, struct quaternion *result)
{
	/*
	  (q0 p0 − q1 p1 − q2 p2 − q3 p3) +
	  (q0 p1 + q1 p0 + q2 p3 − q3 p2)i +
	  (q0 p2 + q2 p0 − q1 p3 + q3 p1)j +
	  (q0 p3 + q3 p0 + q1 p2 − q2 p1)k
	 */
	float q0, q1, q2, q3;
	float p0, p1, p2, p3;
	q0 = a->q_w;
	q1 = a->q_x;
	q2 = a->q_y;
	q3 = a->q_z;
	p0 = b->q_w;
	p1 = b->q_x;
	p2 = b->q_y;
	p3 = b->q_z;

	result->q_w = q0 * p0 - q1 * p1 - q2 * p2 - q3 * p3;
	result->q_x = q0 * p1 + q1 * p0 + q2 * p3 - q3 * p2;
	result->q_y = q0 * p2 + q2 * p0 - q1 * p3 + q3 * p1;
	result->q_z = q0 * p3 + q3 * p0 + q1 * p2 - q2 * p1;
}

void quat_scale(struct quaternion *q, float scalar, struct quaternion *result)
{
	result->q_w = q->q_w * scalar;
	result->q_x = q->q_x * scalar;
	result->q_y = q->q_y * scalar;
	result->q_z = q->q_z * scalar;
}

float quat_norm(struct quaternion *q)
{
	float vec[4] = {q->q_w, q->q_x, q->q_y, q->q_z};
	return quat_vector_norm(vec, 4);
}

void quat_normalize(struct quaternion *q)
{
	float vec[4] = {q->q_w, q->q_x, q->q_y, q->q_z};
	float norm = quat_vector_norm(vec, 4);

	if (norm > QUAT_EPSILON) {

		float scale = 1.0f / norm;
		q->q_w = q->q_w * scale;
		q->q_x = q->q_x * scale;
		q->q_y = q->q_y * scale;
		q->q_z = q->q_z * scale;
	} else {
		quat_identity(q);
	}
}

void quat_conjugate(struct quaternion *q, struct quaternion *result)
{
	result->q_w = q->q_w;
	result->q_x = -q->q_x;
	result->q_y = -q->q_y;
	result->q_z = -q->q_z;
}

void quat_inverse(struct quaternion *q, struct quaternion *result)
{
	/* q⁻¹ = q* / |q|² */
	float norm = quat_norm(q);
	float norm_square = norm * norm;

	if (norm_square > QUAT_EPSILON) {
		float scale = 1.0f / norm_square;
		result->q_w = q->q_w * scale;
		result->q_x = -q->q_x * scale;
		result->q_y = -q->q_y * scale;
		result->q_z = -q->q_z * scale;
	} else {
		quat_identity(result);
	}
}

void quat_derivative(struct quaternion *q, float w[3], struct quaternion *result)
{
	/* dq/dt = 0.5 * q ⊗ ω */
	/* ω = [0, w_x, w_y, w_z] (pure quaterion) */
	float w_x, w_y, w_z, q_w, q_x, q_y, q_z;
	w_x = w[0];
	w_y = w[1];
	w_z = w[2];
	q_w = q->q_w;
	q_x = q->q_x;
	q_y = q->q_y;
	q_z = q->q_z;

	result->q_w = 0.5f * (-w_x * q_x - w_y * q_y - w_z * q_z);
	result->q_x = 0.5f * (w_x * q_w + w_z * q_y - w_y * q_z);
	result->q_y = 0.5f * (w_y * q_w - w_z * q_x + w_x * q_z);
	result->q_z = 0.5f * (w_z * q_w + w_y * q_x - w_x * q_y);
}

void quat_from_axis_angle(float axis[3], float angle, struct quaternion *q)
{
	float half_angle = angle * 0.5f;
	float sin_half, cos_half;
	arm_sin_cos_f32(half_angle, &sin_half, &cos_half);

	/* normalize the ortation axis */
	float axis_norm = quat_vector_norm(axis, 3);
	if (axis_norm > QUAT_EPSILON) {
		float scale = 1.0f / axis_norm;
		q->q_w = cos_half;
		q->q_x = axis[0] * sin_half * scale;
		q->q_y = axis[1] * sin_half * scale;
		q->q_z = axis[2] * sin_half * scale;
	} else {
		/* invalid rotation axie */
		quat_identity(q);
	}
}

void quat_to_axis_angle(struct quaternion *q, float axis[3], float *angle)
{
	float norm = quat_norm(q);

	if (norm > QUAT_EPSILON) {
		/* get angle */
		float q_w_over_norm = q->q_w / norm;
		if (q_w_over_norm > 1.0f) {
			q_w_over_norm = 1.0f;
		} else if (q_w_over_norm < -1.0f) {
			q_w_over_norm = -1.0f;
		}
		*angle = 2 * acosf(q_w_over_norm);

		/* get rotation axis */
		float sin_half_square = 1.0f - (q->q_w * q->q_w) / (norm * norm);
		float sin_half;
		arm_sqrt_f32(sin_half_square, &sin_half);

		if (sin_half > QUAT_EPSILON) {
			float scale = 1.0f / sin_half;
			axis[0] = q->q_x * scale;
			axis[1] = q->q_y * scale;
			axis[2] = q->q_z * scale;
		} else {
			/* 0 or 2 pi, rotation axis is not defined */
			axis[0] = 1.0f;
			axis[1] = 0.0f;
			axis[2] = 0.0f;
		}
	} else {
		/* invalid quaternion */
		axis[0] = 1.0f;
		axis[1] = 0.0f;
		axis[2] = 0.0f;
		*angle = 0.0f;
	}
}

void quat_rotate_vector(struct quaternion *q, float vec[3], float result[3])
{
	/*
	 * q = [s, v'], s = q_w, v' = [q_x, q_y, q_z]
	 * v_rotated = v + 2.0f * cross(v', cross(v', v) + s * v)
	 */
	float s = q->q_w;
	float x = q->q_x;
	float y = q->q_y;
	float z = q->q_z;

	/* t = 2.0f * cross(v', v) */
	float t_x = 2.0f * (y * vec[2] - z * vec[1]);
	float t_y = 2.0f * (z * vec[0] - x * vec[2]);
	float t_z = 2.0f * (x * vec[1] - y * vec[0]);

	/* result = v + cross(v', t) + s * t */
	result[0] = vec[0] + (y * t_z - z * t_y) + s * t_x;
	result[1] = vec[1] + (z * t_x - x * t_z) + s * t_y;
	result[2] = vec[2] + (x * t_y - y * t_x) + s * t_z;

	/* brute force method
	 *
	 * float q0q0 = q_w * q_w;
	 * float q0q1 = q_w * q_x;
	 * float q0q2 = q_w * q_y;
	 * float q0q3 = q_w * q_z;
	 * float q1q1 = q_x * q_x;
	 * float q1q2 = q_x * q_y;
	 * float q1q3 = q_x * q_z;
	 * float q2q2 = q_y * q_y;
	 * float q2q3 = q_y * q_z;
	 * float q3q3 = q_z * q_z;

	   //  R v =
	   // [ 1-2(q_y^2+q_z^2)   2(q1q2-q0q3)     2(q1q3+q0q2)   ] [x]
	   // [ 2(q1q2+q0q3)     1-2(q_x^2+q_z^2)   2(q2q3-q0q1)   ] [y]
	   // [ 2(q1q3-q0q2)     2(q2q3+q0q1)     1-2(q_x^2+q_y^2) ] [z]

	 *  result[0] = v[0] * (q0q0 + q1q1 - q2q2 - q3q3) +
	 *  v[1] * 2.0f * (q1q2 - q0q3) + v[2] * 2.0f * (q1q3 + q0q2);
	 *
	 *  result[1] = v[0] * 2.0f * (q1q2 + q0q3) +
	 *  v[1] * (q0q0 - q1q1 + q2q2 - q3q3) + v[2] * 2.0f * (q2q3 - q0q1);
	 *
	 *  result[2] = v[0] * 2.0f * (q1q3 - q0q2) +
	 *  v[1] * 2.0f * (q2q3 + q0q1) + v[2] * (q0q0 - q1q1 - q2q2 + q3q3);
	 */
}

/* quaternion form euler angles */
/* euler[0]: yaw, euler[1]: pitch, euler[2]: roll */
void quat_from_euler(float euler[3], struct quaternion *q)
{
	;
	/* this may have bug, to be checked */
	/* float roll = euler[2] * 0.5f; */
	/* float pitch = euler[1] * 0.5f; */
	/* float yaw = euler[0] * 0.5f; */

	/* float sr, cr, sp, cp, sy, cy; */

	/* arm_sin_cos_f32(roll, &sr, &cr); */
	/* arm_sin_cos_f32(pitch, &sp, &cp); */
	/* arm_sin_cos_f32(yaw, &sy, &cy); */

	/* q->q_w = cr * cp * cy + sr * sp * sy; */
	/* q->q_x = sr * cp * cy - cr * sp * sy; */
	/* q->q_y = cr * sp * cy + sr * cp * sy; */
	/* q->q_z = cr * cp * sy - sr * sp * cy; */
}

/* from quaternion to euler angles */
/* euler[0]: yaw, euler[1]: pitch, euler[2]: roll */
void quat_to_euler(struct quaternion *q, float euler[3])
{
	float q_w, q_x, q_y, q_z;
	struct quaternion q_norm;
	quat_copy(&q_norm, q);
	quat_normalize(&q_norm);
	q_w = q_norm.q_w;
	q_x = q_norm.q_x;
	q_y = q_norm.q_y;
	q_z = q_norm.q_z;

	/* roll (x-axis rotation) */
	float sinr_cosp = 2.0f * (q_w * q_x + q_y * q_z);
	float cosr_cosp = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);
	euler[2] = atan2f(sinr_cosp, cosr_cosp);

	/* pitch (y-axis rotation) */
	float sinp = 2.0f * (q_w * q_y - q_z * q_x);
	if (sinp > 1.0f) {
		sinp = 1.0f;
	}
	if (sinp < -1.0f) {
		sinp = -1.0f;
	}
	euler[1] = asinf(sinp);

	/* yaw (z-axis rotation) */
	float siny_cosp = 2.0f * (q_w * q_z + q_x * q_y);
	float cosy_cosp = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
	euler[0] = atan2f(siny_cosp, cosy_cosp);
}
