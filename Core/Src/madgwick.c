/*
 * madgwick.c
 *
 *  Created on: May 6, 2021
 *      Author: k-trash
 */

#include "madgwick.h"
#include "math.h"

#define SAMPLE_FREQ_DEF 512.0f          // sample frequency in Hz
#define BETA_DEF 0.1f            // 2 * proportional gain

void beginFilter(float sample_frequency_) {
	filter.beta = betaDef;
	filter.q0 = 1.0f;
	filter.q1 = 0.0f;
	filter.q2 = 0.0f;
	filter.q3 = 0.0f;
	filter.inv_sample_freq = 1.0f / sample_frequency_;
	filter.roll = 0.0f;
	filter.pitch = 0.0f;
	filter.yaw = 0.0f;
	filter.angles_computed = 0;
}

void updateIMU(float gx_, float gy_, float gz_, float ax_, float ay_, float az_){
	float recip_norm;
	float s0, s1, s2, s3;
	float q_dot1, q_dot2, q_dot3, q_dot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	gx_ *= 0.0174533f;
	gy_ *= 0.0174533f;
	gz_ *= 0.0174533f;

	q_dot1 = 0.5f * (-(filter.q1) * gx - filter.q2 * gy - filter.q3 * gz);
	q_dot2 = 0.5f * (filter.q0 * gx + filter.q2 * gz - filter.q3 * gy);
	q_dot3 = 0.5f * (filter.q0 * gy - filter.q1 * gz + filter.q3 * gx);
	q_dot4 = 0.5f * (filter.q0 * gz + filter.q1 * gy - filter.q2 * gx);

	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

	recip_norm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recip_norm;
	ay *= recip_norm;
	az *= recip_norm;

	_2q0 = 2.0f * filter.q0;
	_2q1 = 2.0f * filter.q1;
	_2q2 = 2.0f * filter.q2;
	_2q3 = 2.0f * filter.q3;
	_4q0 = 4.0f * filter.q0;
	_4q1 = 4.0f * filter.q1;
	_4q2 = 4.0f * filter.q2;
	_8q1 = 8.0f * filter.q1;
	_8q2 = 8.0f * filter.q2;
	q0q0 = filter.q0 * filter.q0;
	q1q1 = filter.q1 * filter.q1;
	q2q2 = filter.q2 * filter.q2;
	q3q3 = filter.q3 * filter.q3;

	s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
	s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * filter.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
	s2 = 4.0f * q0q0 * filter.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
	s3 = 4.0f * q1q1 * filter.q3 - _2q1 * ax + 4.0f * q2q2 * filter.q3 - _2q2 * ay;
	recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
	s0 *= recip_norm;
	s1 *= recip_norm;
	s2 *= recip_norm;
	s3 *= recip_norm;

	q_dot1 -= filter.beta * s0;
	q_dot2 -= filter.beta * s1;
	q_dot3 -= filter.beta * s2;
	q_dot4 -= filter.beta * s3;
	}

	filter.q0 += qDot1 * filter.inv_sample_freq;
	filter.q1 += qDot2 * filter.inv_sample_freq;
	filter.q2 += qDot3 * filter.inv_sample_freq;
	filter.q3 += qDot4 * filter.inv_sample_freq;

	recip_norm = invSqrt(filter.q0 * filter.q0 + filter.q1 * filter.q1 + filter.q2 * filter.q2 + filter.q3 * filter.q3);
	filter.q0 *= recip_norm;
	filter.q1 *= recip_norm;
	filter.q2 *= recip_norm;
	filter.q3 *= recip_norm;
	filter.angles_computed = 0;
}

float invSqrt(float x_){
	float halfx = 0.5f * x_;
	float y = x_;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void computeAngles(void){
	filter.roll = atan2f(filter.q0*filter.q1 + filter.q2*filter.q3, 0.5f - filter.q1*filter.q1 - filter.q2*filter.q2);
	filter.pitch = asinf(-2.0f * (filter.q1*filter.q3 - filter.q0*filter.q2));
	filter.yaw = atan2f(filter.q1*filter.q2 + filter.q0*filter.q3, 0.5f - filter.q2*filter.q2 - filter.q3*filter.q3);
	filter.angles_computed = 1;
}

float getRoll(void) {
    if (!filter.angles_computed) computeAngles();
    return filter.roll * 57.29578f;
}

float getPitch(void) {
    if (!filter.angles_computed) computeAngles();
    return filter.pitch * 57.29578f;
}

float getYaw(void){
    if (!filter.angles_computed) computeAngles();
    return filter.yaw * 57.29578f;
}
