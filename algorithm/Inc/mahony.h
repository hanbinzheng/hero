#ifndef MAHONY_H_
#define MAHONY_H_

#include "arm_math.h"
#include "quaternion.h"
#include <stdint.h>

struct mahony_filter {
	struct quaternion q; /* current orientation quaternion */
	float integral[3];   /* integral error */
	float kp;
	float ki;
	float i_limit;
	float sample_freq;
};

void mahony_init(struct mahony_filter *filter, float kp, float ki, float i_limit,
		 float sample_freq);

void mahony_update(struct mahony_filter *filter, float gyro[3], float accel[3]);

struct quaternion *mahony_get_quaternion(struct mahony_filter *filter);
void mahony_get_euler(struct mahony_filter *filter, float euler[3]);

/* assume stabale, then get orientation form g */
void get_orientation_from_g(struct quaternion *q, float euler[3], float accel[3]);

#endif /* MAHONY_H_ */
