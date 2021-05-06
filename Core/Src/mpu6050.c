/*
 * mpu6050.c
 *
 *  Created on: May 6, 2021
 *      Author: k-trash
 */

#include "mpu6050.h"
#include "stm32l4xx_hal.h"

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

I2C_HandleTypeDef hi2c1;

void beginMPU6050(int sample_freq_){
	imu.off_ax = 0.0f;
	imu.off_ay = 0.0f;
	imu.off_az = 0.0f;
	imu.off_gx = 0.0f;
	imu.off_gy = 0.0f;
	imu.off_gz = 0.0f;
	imu.ax = 0.0f;
	imu.ay = 0.0f;
	imu.az = 0.0f;
	imu.gx = 0.0f;
	imu.gy = 0.0f;
	imu.gz = 0.0f;

	uint8_t check;
	uint8_t data;
	float toff_ax, toff_ay, toff_az = 0.0f;
	float toff_gx, toff_gy, toff_gz = 0.0f;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104){
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&data, 1, 1000);
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}

	for(int i = 0; i < sample_freq_; i++){
		updateMPU6050();
		toff_ax += imu.ax;
		toff_ay += imu.ay;
		toff_az += imu.az;
		toff_gx += imu.gx;
		toff_gy += imu.gy;
		toff_gz += imu.gz;
	}
	imu.off_ax = toff_ax / sample_freq_;
	imu.off_ay = toff_ay / sample_freq_;
	imu.off_az = toff_az / sample_freq_ - 1.0f;
	imu.off_gx = toff_gx / sample_freq_;
	imu.off_gy = toff_gy / sample_freq_;
	imu.off_gz = toff_gz / sample_freq_;
}

void updateMPU6050(void){
	volatile uint8_t rec_data_accel[6];
	volatile uint8_t rec_data_gyro[6];
	volatile int16_t ax_raw, ay_raw, az_raw;
	volatile int16_t gx_raw, gy_raw, gz_raw;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, rec_data_accel, 6, 1000);
	ax_raw = (int16_t)(rec_data_accel[0] << 8 | rec_data_accel[1]);
	ay_raw = (int16_t)(rec_data_accel[2] << 8 | rec_data_accel[3]);
	az_raw = (int16_t)(rec_data_accel[4] << 8 | rec_data_accel[5]);
	imu.ax = (ax_raw * 2.0f) / 32768.0f - imu.off_ax;
	imu.ay = (ay_raw * 2.0f) / 32768.0f - imu.off_ay;
	imu.az = (az_raw * 2.0f) / 32768.0f - imu.off_az;

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, rec_data_gyro, 6, 1000);
	gx_raw = (int16_t)(rec_data_gyro[0] << 8 | rec_data_gyro[1]);
	gy_raw = (int16_t)(rec_data_gyro[2] << 8 | rec_data_gyro[3]);
	gz_raw = (int16_t)(rec_data_gyro[4] << 8 | rec_data_gyro[5]);
	imu.gx = (gx_raw * 250.0f) / 32768.0f - imu.off_gx;
	imu.gy = (gy_raw * 250.0f) / 32768.0f - imu.off_gy;
	imu.gz = (gz_raw * 250.0f) / 32768.0f - imu.off_gz;
}

float getAccel_X(void){
	return imu.ax;
}

float getAccel_Y(void){
	return imu.ay;
}

float getAccel_Z(void){
	return imu.az;
}

float getGyro_X(void){
	return imu.gx;
}

float getGyro_Y(void){
	return imu.gy;
}

float getGyro_Z(void){
	return imu.gz;
}
