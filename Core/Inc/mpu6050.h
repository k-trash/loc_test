/*
 * mpu6050.h
 *
 *  Created on: May 6, 2021
 *      Author: k-trash
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stdio.h"

typedef struct Mpu6050{
	volatile float off_ax;
	volatile float off_ay;
	volatile float off_az;
	volatile float off_gx;
	volatile float off_gy;
	volatile float off_gz;
	volatile float ax;
	volatile float ay;
	volatile float az;
	volatile float gx;
	volatile float gy;
	volatile float gz;
}Mpu6050;

Mpu6050 imu;

void beginMPU6050(int sample_freq_);

void updateMPU6050(void);

float getAccelX(void);
float getAccelY(void);
float getAccelZ(void);

float getGyroX(void);
float getGyroY(void);
float getGyroZ(void);

#endif /* INC_MPU6050_H_ */
