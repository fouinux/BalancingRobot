/*
 * ADXL345.c
 *
 *  Created on: 20 mars 2018
 *      Author: fouinux@gmail.com
 */

#include <stdint.h>
#include <math.h>
#include <stm32f3xx_hal.h>
#include "Drivers/ADXL345.h"

#include "Tools/Log.h"

extern I2C_HandleTypeDef hi2c1;

static float gGain = ADXL345_GAIN_FULL_RESOLUTION; /* Default gain value : full resolution */

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
		LOG_DEBUG("DEVID = 0x%02x\r\n", RegValue);

		if (RegValue != ADXL345_DEVID)
		{
			Ret = HAL_ERROR;
		}
		else
		{
			/* Init ADXL345 */
			do
			{
				/* Reset offset registers */
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSX, 0);
				if (Ret) break;
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSY, 0);
				if (Ret) break;
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSZ, 0);
				if (Ret) break;

				/* Select range and full resolution */
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_DATA_FORMAT,
						ADXL345_DATA_FORMAT_BM_FULL_RES |
						ADXL345_DATA_FORMAT_BM_RANGE_16G);
				if (Ret) break;

				gGain = ADXL345_GAIN_FULL_RESOLUTION;

				/* No low power and output rate 100Hz */
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_BW_RATE,
						ADXL345_BW_RATE_BM_RATE_100);
				if (Ret) break;

				/* FIFO in bypass mode */
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_FIFO_CTL,
						ADXL345_FIFO_CTL_BM_FIFO_MODE_BYPASS);
				if (Ret) break;

				/* Start measurement */
				Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_POWER_CTL,
						ADXL345_POWER_CTL_BM_MEASURE);
				if (Ret) break;
			} while(0);
		}
	}


	return Ret;
}

uint16_t ADXL345_GetDataRaw(int16_t *pX, int16_t *pY, int16_t *pZ)
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

		LOG_DEBUG("A raw: X = %+5d, Y = %+5d, Z = %+5d\r\n",
				*((int16_t *) &aBuffer[0]),
				*((int16_t *) &aBuffer[2]),
				*((int16_t *) &aBuffer[4]));
	}

	return Ret;
}


uint16_t ADXL345_GetData(float *pX_g, float *pY_g, float *pZ_g)
{
	uint16_t Ret = 0;
	int16_t aBuffer[3];

	/* Read raw X, Y and Z */
	Ret = ADXL345_GetDataRaw(&aBuffer[0], &aBuffer[1], &aBuffer[2]);
	if (Ret == HAL_OK)
	{
		if (pX_g != NULL)
			*pX_g = (float) aBuffer[0] * gGain;

		if (pY_g != NULL)
			*pY_g = (float) aBuffer[1] * gGain;

		if (pZ_g != NULL)
			*pZ_g = (float) aBuffer[2] * gGain;

		LOG_DEBUG("A: X = %+1.3fg, Y = %+1.3fg, Z = %+1.3fg\r\n",
				aBuffer[0] * gGain,
				aBuffer[1] * gGain,
				aBuffer[2] * gGain);
	}

	return Ret;
}


#define OFFSET_CALIBRATION_AVG_NB		10
#define OFFSET_CALIBRATION_DELAY		10
uint16_t ADXL345_OffsetCalibration(void)
{
	uint16_t Ret = 0;
	float aBuffer[3];
	float AvgX = 0.0, AvgY = 0.0, AvgZ = 0.0;

	do
	{
		for (uint8_t i = 0 ; i < OFFSET_CALIBRATION_AVG_NB ; i++)
		{
			HAL_Delay(OFFSET_CALIBRATION_DELAY);
			Ret = ADXL345_GetData(&aBuffer[0], &aBuffer[1], &aBuffer[2]);
			if (Ret) break;

			AvgX += aBuffer[0];
			AvgY += aBuffer[1];
			AvgZ += aBuffer[2];
		}

		if (!Ret)
		{
			/* Compute average */
			AvgX /= (float) OFFSET_CALIBRATION_AVG_NB;
			AvgY /= (float) OFFSET_CALIBRATION_AVG_NB;
			AvgZ /= (float) OFFSET_CALIBRATION_AVG_NB;

			/* Assume Z is +1g */
			AvgZ -= 1.0;

			/* Compute offset */
			int8_t OffsetX = -round(AvgX / ADXL345_GAIN_OFFSET);
			int8_t OffsetY = -round(AvgY / ADXL345_GAIN_OFFSET);
			int8_t OffsetZ = -round(AvgZ / ADXL345_GAIN_OFFSET);

			Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSX, OffsetX);
			if (Ret) break;
			Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSY, OffsetY);
			if (Ret) break;
			Ret = ADXL345_WriteReg(ADXL345_REG_ADDR_OFSZ, OffsetZ);
			if (Ret) break;

			LOG_DEBUG("A offset calibration: X = %+d, Y = %+d, Z = %+d\r\n",
					OffsetX, OffsetY, OffsetZ);
		}
	} while (0);

	return Ret;
}
