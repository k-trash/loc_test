/*
 * madgwick.h
 *
 *  Created on: May 6, 2021
 *      Author: k-trash
 */

#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

typedef struct Madgwick{
	float beta;
	float q0;
	float q1;
	float q2;
	float q3;
	float inv_sample_freq;
	float roll;
	float pitch;
	float yaw;
	char angles_computed;
}Madgwick;

Madgwick filter;

float invSqrt(float x_);

void beginFilter(float sample_freq_);

void updateIMU(float gx_, float gy_, float gz_, float ax_, float ay_, float az_);

void computeAngles(void);

float getRoll(void);
float getPitch(void);
float getYaw(void);

#endif /* INC_MADGWICK_H_ */
