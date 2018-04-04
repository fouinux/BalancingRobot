/*
 * IMU.h
 *
 *  Created on: 03 april 2018
 *      Author: fouinux@gmail.com
 */
#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include <inttypes.h>

/* Public functions */
uint16_t IMU_Init(void);
uint16_t IMU_GetOrientation(float *pYaw, float *pPitch, float *pRoll);

#endif /* IMU_IMU_H_ */
