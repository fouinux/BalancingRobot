/*
 * IMU.h
 *
 *  Created on: 03 april 2018
 *      Author: fouinux@gmail.com
 */
#include "IMU/IMU.h"
#include <math.h>

#include "IMU/AHRS.h"

#include "Drivers/ADXL345.h"
#include "Drivers/L3G4200D.h"
#include "Drivers/HMC5883L.h"

void GetEulerAngle(float *pRoll, float *pPitch, float *pYaw)
{
	// roll (x-axis rotation)
	float sinr = +2.0 * (q0 * q1 + q2 * q3);
	float cosr = +1.0 - 2.0 * (q1 * q1 + q2 * q2);
	*pRoll = atan2f(sinr, cosr);

	// pitch (y-axis rotation)
	float sinp = +2.0 * (q0 * q2 - q3 * q1);
	if (fabsf(sinp) >= 1)
		*pPitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		*pPitch = asinf(sinp);

	// yaw (z-axis rotation)
	float siny = +2.0 * (q0 * q3 + q1 * q2);
	float cosy = +1.0 - 2.0 * (q2 * q2 + q3 * q3);
	*pYaw = atan2f(siny, cosy);
}

uint16_t IMU_Init(void)
{
	uint16_t Ret = 0;

	/* Init accelerometer */
	Ret = ADXL345_Init();
	if (Ret) return Ret;

	/* Init gyroscope */
	Ret = L3G4200D_Init();
	if (Ret) return Ret;

	/* Init magnetometer */
	Ret = HMC5883L_Init();
	if (Ret) return Ret;

	/* Perform offset calibration */
	Ret = ADXL345_OffsetCalibration();
	if (Ret) return Ret;

	return Ret;
}

uint16_t IMU_GetOrientation(float *pYaw, float *pPitch, float *pRoll)
{
	uint16_t Ret = 0;
	float	Ax, Ay, Az;
//	float 	Gx, Gy, Gz;
//	float 	Mx, My, Mz;

	Ret = ADXL345_GetData(&Ax, &Ay, &Az);
//	Ret = L3G4200D_GetData(&Gx, &Gy, &Gz);
//	Ret = HMC5883L_GetData(&Mx, &My, &Mz);

	return Ret;
}
