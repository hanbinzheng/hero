#include "mahony.h"
#include "quaternion.h"

#define MAHONY_EPSILON (1.0e-6f)

static inline float val_limit_float(float value, float min, float max)
{
	if (value >= max) {
		return max;
	} else if (value <= min) {
		return min;
	} else {
		return value;
	}
}

static inline void normalize_vector(float vec[3])
{

	// arm_dot_prod_f32(vec, vec, 3 &norm_square);
	float norm_square, norm;
	norm_square = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];

	// check and normalize
	if (norm_square > MAHONY_EPSILON * MAHONY_EPSILON) {
		arm_sqrt_f32(norm_square, &norm);
		float scale = 1.0f / norm;

		vec[0] *= scale;
		vec[1] *= scale;
		vec[2] *= scale;
	}
}

static inline void cross_product(float a[3], float b[3], float result[3])
{
	result[0] = a[1] * b[2] - a[2] * b[1];
	result[1] = a[2] * b[0] - a[0] * b[2];
	result[2] = a[0] * b[1] - a[1] * b[0];
}

void mahony_compute_error(struct mahony_filter *filter, float measured[3],
			  float error[3])
{
	// normalize measured acceleration
	float _measured[3] = {measured[0], measured[1], measured[2]};
	normalize_vector(_measured);

	// estimate orientation of g based on current quaternion
	float estimated_g[3];
	static float world_g[3] = {0.0f, 0.0f,
				   1.0f}; // word coordinate gravity orientation
	struct quaternion q_conj;
	quat_conjugate(&(filter->q), &q_conj);
	quat_rotate_vector(&q_conj, world_g,
			   estimated_g); // transform word frame into imu frame

	// acceleration error: cross product (measured x estimated)
	cross_product(_measured, estimated_g, error);
}

void mahony_init(struct mahony_filter *filter, float kp, float ki, float i_limit,
		 float sample_freq)
{
	quat_identity(&(filter->q));
	memset(filter->integral, 0, sizeof(filter->integral));

	filter->kp = kp;
	filter->ki = ki;
	filter->i_limit = i_limit;
	filter->sample_freq = sample_freq;
}

void mahony_update(struct mahony_filter *filter, float gyro[3], float accel[3])
{
	// integtral time scale
	float time_scale = 1.0f / filter->sample_freq;
	float kp = filter->kp;
	float ki = filter->ki;

	// compute error
	float error[3];
	mahony_compute_error(filter, accel, error);

	// compute w_corrected
	float w_corrected[3];
	for (int i = 0; i < 3; i++) {
		// update integral term
		filter->integral[i] += error[i];
		filter->integral[i] = val_limit_float(
		    filter->integral[i], -filter->i_limit, filter->i_limit);

		// w_correct = w + kp * error + ki * \int error
		w_corrected[i] = gyro[i] + kp * error[i] + ki * filter->integral[i];
	}

	// get quaternion derivative
	struct quaternion q_dot;
	quat_derivative(&(filter->q), w_corrected, &q_dot);

	// euler method to update quaternion
	(filter->q).q_w += q_dot.q_w * time_scale;
	(filter->q).q_x += q_dot.q_x * time_scale;
	(filter->q).q_y += q_dot.q_y * time_scale;
	(filter->q).q_z += q_dot.q_z * time_scale;

	// normalize
	quat_normalize(&filter->q);
}

struct quaternion *mahony_get_quaternion(struct mahony_filter *filter)
{
	return &(filter->q);
}

void mahony_get_euler(struct mahony_filter *filter, float euler[3])
{
	quat_to_euler(&(filter->q), euler);
}

/* void get_orientation_from_g(struct quaternion *q, float euler[3], float
 * accel[3]) */
/* { */
/*     float _accel[3] = {accel[0], accel[1], accel[2]}; */
/*     normalize_vector(_accel); */

/*     float a_x, a_y, a_z; */
/*     a_x = _accel[0]; */
/*     a_y = _accel[1]; */
/*     a_z = _accel[2]; */

/*     // pitch */
/*     euler[1] = -atan2f(a_x, sqrtf(a_y * a_y + a_z * a_z)); */
/*     // Roll */
/*     euler[2] = atan2f(a_y, a_z); */
/*     // Yaw: unknown */
/*     euler[0] = 0.0f; */

/*     quat_from_euler(euler, q); */
/* } */
