/*
 * ADXL345.c
 *
 *  Created on: 20 mars 2018
 *      Author: fouinux@gmail.com
 */

#include <stdint.h>
#include <stm32f3xx_hal.h>
#include "Drivers/ADXL345.h"

extern I2C_HandleTypeDef hi2c1;

static uint16_t ADXL345_ReadReg(uint8_t RegAddr, uint8_t *pRegValue)
{
	return HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, RegAddr, I2C_MEMADD_SIZE_8BIT, pRegValue, 1, 100);
}

static uint16_t ADXL345_ReadMultipleReg(uint8_t RegAddr, uint8_t *pRegValue, uint16_t Size)
{
	return HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, RegAddr, I2C_MEMADD_SIZE_8BIT, pRegValue, Size, 100);
}

static uint16_t ADXL345_WriteReg(uint8_t RegAddr, uint8_t RegValue)
{
	uint8_t aBuffer[2] = {RegAddr, RegValue};

	return HAL_I2C_Master_Transmit(&hi2c1, ADXL345_I2C_ADDR, &aBuffer[0], 2, 100);
}

uint16_t ADXL345_Init(void)
{
	uint16_t Ret = 0;
	uint8_t RegValue = 0;

	/* Read ID */
	Ret = ADXL345_ReadReg(ADXL345_REG_ADDR_DEVID, &RegValue);
	if (Ret == HAL_OK)
	{
		printf("DEVID = 0x%02x\r\n", RegValue);

		if (RegValue != ADXL345_DEVID)
		{
			Ret = HAL_ERROR;
		}
		else
		{
			/* Init ADXL345 */

			/* Select range and full resolution */
			Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_DATA_FORMAT, ADXL345_DATA_FORMAT_BM_FULL_RES | ADXL345_DATA_FORMAT_BM_RANGE_16G);

			/* Start measurement */
			Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_POWER_CTL, 0x08);
		}
	}


	return Ret;
}


uint16_t ADXL345_GetAcceleration(int16_t *pX, int16_t *pY, int16_t *pZ)
{
	uint16_t Ret = 0;
	uint8_t aBuffer[6];

	/* Read X, Y and Z accelerations */
	Ret = ADXL345_ReadMultipleReg(ADXL345_REG_ADDR_DATAX0, &aBuffer[0], 6);
	if (Ret == HAL_OK)
	{
		if (pX != NULL)
			*pX = *((int16_t *) &aBuffer[0]);

		if (pY != NULL)
			*pY = *((int16_t *) &aBuffer[2]);

		if (pZ != NULL)
			*pZ = *((int16_t *) &aBuffer[4]);

		printf("Accel: X = %+1.3f, Y = %+1.3f, Z = %+1.3f\r\n",
				*((int16_t *) &aBuffer[0]) * 0.004,
				*((int16_t *) &aBuffer[2]) * 0.004,
				*((int16_t *) &aBuffer[4]) * 0.004);
	}

	return Ret;
}
