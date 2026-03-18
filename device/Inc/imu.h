#ifndef IMU_H_
#define IMU_H_

#include "mahony.h"
#include "quaternion.h"
#include <stdint.h>

struct imu_raw_data {
	float accel[3];
	float gyro[3];
	float temp;
};

struct imu_data {

	float pos_roll;
	float pos_pitch;
	float pos_yaw;

	float vel_roll;
	float vel_pitch;
	float vel_yaw;

	struct quaternion q;
	struct imu_raw_data raw_data;
};

enum bmi_error {
	BMI088_NO_ERROR = 0x00,

	BMI088_GYRO_RANGE_ERROR = 0x01,
	BMI088_GYRO_BANDWIDTH_ERROR = 0x02,
	BMI088_GYRO_LPM1_ERROR = 0x03,

	BMI088_ACC_RANGE_ERROR = 0x04,
	BMI088_ACC_CONF_ERROR = 0x05,
	BMI088_ACC_PWR_CTRL_ERROR = 0x06,
	BMI088_ACC_PWR_CONF_ERROR = 0x07,

	BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
	BMI088_SELF_TEST_GYRO_ERROR = 0x40,
	BMI088_NO_SENSOR = 0xFF,
};

uint8_t imu_init(void);
void imu_update(void);

extern struct imu_data imu_data;
extern struct mahony_filter mahony_filter;

#endif /* IMU_H_ */
