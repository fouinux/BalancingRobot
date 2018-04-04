/*
 * HMC5883L.c
 *
 *  Created on: 29 mars 2018
 *      Author: fouinux@gmail.com
 */

#include <stdint.h>
#include <string.h>
#include <stm32f3xx_hal.h>
#include "Drivers/HMC5883L.h"

#include "Tools/Log.h"

extern I2C_HandleTypeDef hi2c1;

static float gGain = HMC5883L_GAIN_1_30G; /* Default gain value : 1.3 Gauss */

//static uint16_t HMC5883L_ReadReg(uint8_t RegAddr, uint8_t *pRegValue)
//{
//	return HAL_I2C_Mem_Read(&hi2c1, HMC5883L_I2C_ADDR, RegAddr, I2C_MEMADD_SIZE_8BIT, pRegValue, 1, 100);
//}

static uint16_t HMC5883L_ReadMultipleReg(uint8_t RegAddr, uint8_t *pRegValue, uint16_t Size)
{
	return HAL_I2C_Mem_Read(&hi2c1, HMC5883L_I2C_ADDR, RegAddr, I2C_MEMADD_SIZE_8BIT, pRegValue, Size, 100);
}

static uint16_t HMC5883L_WriteReg(uint8_t RegAddr, uint8_t RegValue)
{
	uint8_t aBuffer[2] = {RegAddr, RegValue};

	return HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_I2C_ADDR, &aBuffer[0], 2, 100);
}

uint16_t HMC5883L_Init(void)
{
	uint16_t Ret = 0;
	uint8_t aBuffer[3];
	const uint8_t aExpectedId[] = "\x48\x34\x33";

	/* Read ID */
	Ret = HMC5883L_ReadMultipleReg(HMC5883L_REG_ADDR_ID_REG_A, &aBuffer[0], 3);
	if (Ret == HAL_OK)
	{
		LOG_DEBUG("HMC5883L Id = %02x%02x%02x\r\n", aBuffer[0], aBuffer[1], aBuffer[2]);
		if (memcmp(&aBuffer[0], &aExpectedId[0], 3) != 0)
		{
			Ret = HAL_ERROR;
		}
		else
		{
			/* Init HMC5883L */
			do
			{
				/* Select average 8 samples, 15 Hz output rate and normal mode */
				Ret = HMC5883L_WriteReg(HMC5883L_REG_ADDR_CFG_REG_A,
						HMC5883L_CFG_REG_A_BM_MA_8 			|
						HMC5883L_CFG_REG_A_BM_DO_15_0 		|
						HMC5883L_CFG_REG_A_BM_MS_NOR);
				if (Ret) break;

				/* Select gain to 1.3 Gauss */
				Ret = HMC5883L_WriteReg(HMC5883L_REG_ADDR_CFG_REG_B,
						HMC5883L_CFG_REG_B_BM_GAIN_1_30G);
				if (Ret) break;

				gGain = HMC5883L_GAIN_1_30G;

				/* Select continuous mode */
				Ret = HMC5883L_WriteReg(HMC5883L_REG_ADDR_MODE_REG,
						HMC5883L_MODE_REG_BM_MD_CONTINUOUS);
				if (Ret) break;
			} while (0);
		}
	}

	return Ret;
}

uint16_t HMC5883L_GetDataRaw(int16_t *pX, int16_t *pY, int16_t *pZ)
{
	uint16_t Ret = 0;
	uint8_t aBuffer[6];

	/* Read X, Y and Z */
	Ret = HMC5883L_ReadMultipleReg(HMC5883L_REG_ADDR_DATA_OUT_X_MSB, &aBuffer[0], 6);
	if (Ret == HAL_OK)
	{
		if (pX != NULL)
			*pX = ((uint16_t) aBuffer[0] << 8) | ((uint16_t) aBuffer[1]);

		if (pY != NULL)
			*pY = ((uint16_t) aBuffer[2] << 8) | ((uint16_t) aBuffer[3]);

		if (pZ != NULL)
			*pZ = ((uint16_t) aBuffer[4] << 8) | ((uint16_t) aBuffer[5]);

		LOG_DEBUG("M raw: X =%+5d, Y = %+5d, Z = %+5d\r\n",
				(int16_t)(((int16_t) aBuffer[0] << 8) | ((int16_t) aBuffer[1])),
				(int16_t)(((int16_t) aBuffer[2] << 8) | ((int16_t) aBuffer[3])),
				(int16_t)(((int16_t) aBuffer[4] << 8) | ((int16_t) aBuffer[5])));
	}

	return Ret;
}

uint16_t HMC5883L_GetData(float *pX_gauss, float *pY_gauss, float *pZ_gauss)
{
	uint16_t Ret = 0;
	int16_t aBuffer[3];

	/* Read X, Y and Z in raw */
	Ret = HMC5883L_GetDataRaw(&aBuffer[0], &aBuffer[1], &aBuffer[2]);
	if (Ret == HAL_OK)
	{
		if (pX_gauss != NULL)
			*pX_gauss = (float) aBuffer[0] * 0.00092;

		if (pY_gauss != NULL)
			*pY_gauss = (float) aBuffer[1] * 0.00092;

		if (pZ_gauss != NULL)
			*pZ_gauss = (float) aBuffer[2] * 0.00092;

		LOG_DEBUG("M: X = %+1.3f, Y = %+1.3f, Z = %+1.3f\r\n",
				(float) aBuffer[0] * gGain,
				(float) aBuffer[1] * gGain,
				(float) aBuffer[2] * gGain);
	}

	return Ret;
}
