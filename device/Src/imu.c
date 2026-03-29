#include "imu.h"
#include "bmi088_reg.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define GYRO_SENSITIVITY_1000 32.768f
#define BMI088_GYRO_2000_SEN (0.00106526443603169529841533860381f)
#define ACCEL_SENSITIVITY_3 10922.66667f

#define TWO_PI_OVER_180 (2 * 3.14159265358979f / 180.0f)

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_COM_WAIT_SENSOR_TIME 150 /* communication wait time: 150us */
#define BMI088_LONG_DELAY_TIME 100	/* long delay time: 100ms */

/*
 **************************************************************************
 * global variables and constants
 **************************************************************************
 */
struct imu_data imu_data = {
    .pos_roll = 0.0f,
    .pos_pitch = 0.0f,
    .pos_yaw = 0.0f,
    .vel_roll = 0.0f,
    .vel_pitch = 0.0f,
    .vel_yaw = 0.0f,
    .raw_data = {.accel = {0}, .gyro = {0}, .temp = 0},
    .q = {.q_w = 1.0f, .q_x = 0.0f, .q_y = 0.0f, .q_z = 0.0f},
};

struct mahony_filter mahony_filter = {
    .q = {.q_w = 1.0f, .q_x = 0.0f, .q_y = 0.0f, .q_z = 0.0f},
    .integral = {0.0f, 0.0f, 0.0f},
    .kp = 0.6f,
    .ki = 0.0f,
    .i_limit = 0.0f,
    .sample_freq = 1000.0f,
};

/* experimental gyro bias */
float gyro_bias[3] = {0.00398518f, 0.00122815f, 0.00283814f};

/*
 **************************************************************************
 * BMI088 basic data interpretation
 **************************************************************************
 */
static inline void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
	SET_CS_ACCEL_LOW();
	spi_tx_byte(&hspi2, data);
	SET_CS_ACCEL_HIGH();
}

static inline void bmi088_accel_read_single_reg(uint8_t reg, uint8_t *data)
{
	SET_CS_ACCEL_LOW();
	spi_tx_byte(&hspi2, reg | 0x80);
	spi_tx_byte(&hspi2, 0x55); /* dummy write */
	*data = spi_tx_byte(&hspi2, 0x55);
	SET_CS_ACCEL_HIGH();
}

static inline void bmi088_accel_read_multi_reg(uint8_t reg, uint8_t *rx_buf)
{
	SET_CS_ACCEL_LOW();
	spi_tx_byte(&hspi2, reg | 0x80);
	spi_tx_byte(&hspi2, 0x55); /* dummy write */
	for (uint8_t i = 0; i < 6; i++) {
		rx_buf[i] = spi_tx_byte(&hspi2, 0x55);
	}
	SET_CS_ACCEL_HIGH();
}

static inline void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
	SET_CS_GYRO_LOW();
	spi_tx_byte(&hspi2, reg | 0x7F);
	spi_tx_byte(&hspi2, data);
	SET_CS_GYRO_HIGH();
}

static inline void bmi088_gyro_read_single_reg(uint8_t reg, uint8_t *data)
{
	SET_CS_GYRO_LOW();
	spi_tx_byte(&hspi2, reg | 0x80);
	*data = spi_tx_byte(&hspi2, 0x55);
	SET_CS_GYRO_HIGH();
}

static inline void bmi088_gyro_read_multi_reg(uint8_t reg, uint8_t *rx_buf)
{
	SET_CS_GYRO_LOW();
	spi_tx_byte(&hspi2, reg | 0x80);
	for (uint8_t i = 0; i < 6; i++) {
		rx_buf[i] = spi_tx_byte(&hspi2, 0x55);
	}
	SET_CS_GYRO_HIGH();
}

uint8_t bmi088_accel_init(void)
{
	/* configure, set registers and define corresponding errors */
	uint8_t BMI088_Acc_Init_Config[4][3] = {
	    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
	    {BMI088_ACC_CONF, BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
	     BMI088_ACC_CONF_ERROR},
	    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
	     BMI088_ACC_PWR_CTRL_ERROR},
	    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
	     BMI088_ACC_PWR_CONF_ERROR}};
	static uint8_t read_value;

	/* software reset, required */
	bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET,
				      BMI088_ACC_SOFTRESET_VALUE);
	dwt_delay_ms(BMI088_LONG_DELAY_TIME);
	bmi088_accel_write_single_reg(BMI088_ACC_PWR_CONF,
				      BMI088_ACC_PWR_ACTIVE_MODE);
	dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	/* check if commiunication is normal after reset */
	bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &read_value);
	dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	/* check "who am I" */
	if (read_value != BMI088_ACC_CHIP_ID_VALUE) {
		return BMI088_NO_SENSOR; /* ??? */
	}

	/* write register */
	for (uint8_t i = 0; i < 4; i++) {
		bmi088_accel_write_single_reg(BMI088_Acc_Init_Config[i][0],
					      BMI088_Acc_Init_Config[i][1]);
		dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		bmi088_accel_read_single_reg(BMI088_Acc_Init_Config[i][0],
					     &read_value);
		dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		/* return if error occurs */
		if (read_value != BMI088_Acc_Init_Config[i][1]) {
			return BMI088_Acc_Init_Config[i][2];
		}
	}
	return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
	uint8_t BMI088_Gyro_Init_Config[3][3] = {
	    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
	    {BMI088_GYRO_BANDWIDTH,
	     BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
	     BMI088_GYRO_BANDWIDTH_ERROR},
	    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR}};
	static uint8_t read_value;

	bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET,
				     BMI088_GYRO_SOFTRESET_VALUE);
	dwt_delay_ms(BMI088_LONG_DELAY_TIME);

	/* check whether communication is normal */
	bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &read_value);
	dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

	/* who am I */
	if (read_value != BMI088_GYRO_CHIP_ID_VALUE) {
		return BMI088_NO_SENSOR;
	}

	/* configure registers */
	for (uint8_t i = 0; i < 3; i++) {
		bmi088_gyro_write_single_reg(BMI088_Gyro_Init_Config[i][0],
					     BMI088_Gyro_Init_Config[i][1]);
		dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		bmi088_gyro_read_single_reg(BMI088_Gyro_Init_Config[i][0],
					    &read_value);
		dwt_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
		/* return if error occurs */
		if (read_value != BMI088_Gyro_Init_Config[i][1]) {
			return BMI088_Gyro_Init_Config[i][2];
		}
	}
	return BMI088_NO_ERROR;
}

uint8_t imu_init(void)
{
	uint8_t state = BMI088_NO_ERROR;
	state |= bmi088_gyro_init();
	state |= bmi088_accel_init();

	return state;
}

void bmi088_get_temperature(uint8_t *tempbuff)
{
	uint8_t bmi_tx_byte, len = 2;
	SET_CS_ACCEL_LOW();
	bmi_tx_byte = ((BMI088_TEMP_M) | 0x80);

	spi_tx_byte(&hspi2, bmi_tx_byte);
	spi_tx_byte(&hspi2, bmi_tx_byte); /* dummy byte */

	while (len != 0) {
		tempbuff[2 - len] = spi_tx_byte(&hspi2, bmi_tx_byte);
		len--;
	}

	SET_CS_ACCEL_HIGH();
}

void imu_get_data(struct imu_raw_data *data)
{
	uint8_t gyro_buff[6];
	uint8_t accel_buff[6];
	uint8_t temp_buff[2];
	int16_t tmp;

	/* read gyro data, in radius */
	bmi088_gyro_read_multi_reg(BMI088_GYRO_X_L, gyro_buff);
	tmp = (int16_t)((gyro_buff[1] << 8) | gyro_buff[0]);
	data->gyro[0] =
	    (float)tmp * BMI088_GYRO_2000_SEN - gyro_bias[0];
	tmp = (int16_t)((gyro_buff[3] << 8) | gyro_buff[2]);
	data->gyro[1] =
	    (float)tmp * BMI088_GYRO_2000_SEN - gyro_bias[1];
	tmp = (int16_t)((gyro_buff[5] << 8) | gyro_buff[4]);
	data->gyro[2] =
	    (float)tmp * BMI088_GYRO_2000_SEN - gyro_bias[2];

	/* read accel data, the unit is g */
	bmi088_accel_read_multi_reg(BMI088_ACCEL_XOUT_L, accel_buff);
	tmp = (int16_t)((accel_buff[1] << 8) | accel_buff[0]);
	data->accel[0] = ((float)tmp / ACCEL_SENSITIVITY_3);
	tmp = (int16_t)((accel_buff[3] << 8) | accel_buff[2]);
	data->accel[1] = ((float)tmp / ACCEL_SENSITIVITY_3);
	tmp = (int16_t)((accel_buff[5] << 8) | accel_buff[4]);
	data->accel[2] = ((float)tmp / ACCEL_SENSITIVITY_3);

	/* read temperature data */
	bmi088_get_temperature(temp_buff);
	tmp = (int16_t)((temp_buff[0] << 3) | (temp_buff[1] >> 5));
	if (tmp > 1023) {
		tmp -= 2048;
	}
	data->temp = (float)tmp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/*
 **************************************************************************
 * data update implementation
 **************************************************************************
 */
void imu_update(void)
{
	/*  angular velocity under world frame and euler angles */
	float32_t w[3];
	float32_t euler[3];

	/* update raw data */
	imu_get_data(&(imu_data.raw_data));

	/* update velocity */
	quat_rotate_vector(&(imu_data.q), imu_data.raw_data.gyro, w);
	imu_data.vel_roll = w[0];
	imu_data.vel_pitch = w[1];
	imu_data.vel_yaw = w[2];

	/* update quaternion with mahony */
	mahony_update(&mahony_filter, imu_data.raw_data.gyro,
		      imu_data.raw_data.accel);
	imu_data.q = mahony_filter.q;

	/* update euler angles from quaternion*/
	quat_to_euler(&(imu_data.q), euler);
	imu_data.pos_roll = euler[2];
	imu_data.pos_pitch = euler[1];
	imu_data.pos_yaw = euler[0];
}
