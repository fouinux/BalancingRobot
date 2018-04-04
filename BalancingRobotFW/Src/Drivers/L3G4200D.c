/*
 * L3G4200D.c
 *
 *  Created on: 27 mars 2018
 *      Author: fouinux@gmail.com
 */


#include <stdint.h>
#include <math.h>
#include <stm32f3xx_hal.h>
#include "Drivers/L3G4200D.h"

#include "Tools/Log.h"

extern I2C_HandleTypeDef hi2c1;

static uint16_t L3G4200D_ReadReg(uint8_t RegAddr, uint8_t *pRegValue)
{
	return HAL_I2C_Mem_Read(&hi2c1, L3G4200D_I2C_ADDR, RegAddr, I2C_MEMADD_SIZE_8BIT, pRegValue, 1, 100);
}

static uint16_t L3G4200D_ReadMultipleReg(uint8_t RegAddr, uint8_t *pRegValue, uint16_t Size)
{
	return HAL_I2C_Mem_Read(&hi2c1, L3G4200D_I2C_ADDR, RegAddr | L3G4200D_I2C_AUTOINCREMENT_MASK, I2C_MEMADD_SIZE_8BIT, pRegValue, Size, 100);
}

static uint16_t L3G4200D_WriteReg(uint8_t RegAddr, uint8_t RegValue)
{
	uint8_t aBuffer[2] = {RegAddr, RegValue};

	return HAL_I2C_Master_Transmit(&hi2c1, L3G4200D_I2C_ADDR, &aBuffer[0], 2, 100);
}

uint16_t L3G4200D_Init(void)
{
	uint16_t Ret = 0;
	uint8_t RegValue = 0;

	/* Read WHO_AM_I */
	Ret = L3G4200D_ReadReg(L3G4200D_REG_ADDR_WHO_AM_I, &RegValue);
	if (Ret == HAL_OK)
	{
		LOG_DEBUG("WHO_AM_I = 0x%02x\r\n", RegValue);

		if (RegValue != L3G4200D_WHO_AM_I)
		{
			Ret = HAL_ERROR;
		}
		else
		{
			/* Init L3G4200D */
			do
			{
				/* Select Datarate, Bandwidth and normal mode */
				Ret = L3G4200D_WriteReg(L3G4200D_REG_ADDR_CTRL_REG1,
						L3G4200D_CTRL_REG1_BM_DR_100HZ 			|
						(0x00 << L3G4200D_CTRL_REG1_BN_BW) 		|
						L3G4200D_CTRL_REG1_BM_PD 				|
						L3G4200D_CTRL_REG1_BM_Z_EN 				|
						L3G4200D_CTRL_REG1_BM_Y_EN 				|
						L3G4200D_CTRL_REG1_BM_X_EN);
				if (Ret) break;

				/* Select high pass filter mode */
				Ret = L3G4200D_WriteReg(L3G4200D_REG_ADDR_CTRL_REG2, (0x01 << L3G4200D_CTRL_REG2_BN_HPM));
				if (Ret) break;

				/* Select BDU On, Little endian, 250 DPS */
				Ret = L3G4200D_WriteReg(L3G4200D_REG_ADDR_CTRL_REG4,
						L3G4200D_CTRL_REG4_BM_BDU_ON 		|
						L3G4200D_CTRL_REG4_BM_BLE_LITTLE 	|
						L3G4200D_CTRL_REG4_BM_FS_250DPS);
				if (Ret) break;

				/* Select FIFO disabled */
				Ret = L3G4200D_WriteReg(L3G4200D_REG_ADDR_CTRL_REG5, L3G4200D_CTRL_REG5_BM_FIFO_EN_DIS);
				if (Ret) break;
			} while (0);
		}
	}

	return Ret;
}

uint16_t L3G4200D_GetDataRaw(int16_t *pX, int16_t *pY, int16_t *pZ)
{
	uint16_t Ret = 0;
	uint8_t aBuffer[6];

	/* Read X, Y and Z */
	Ret = L3G4200D_ReadMultipleReg(L3G4200D_REG_ADDR_OUT_X_L, &aBuffer[0], 6);
	if (Ret == HAL_OK)
	{
		if (pX != NULL)
			*pX = *((int16_t *) &aBuffer[0]);

		if (pY != NULL)
			*pY = *((int16_t *) &aBuffer[2]);

		if (pZ != NULL)
			*pZ = *((int16_t *) &aBuffer[4]);

		LOG_DEBUG("G raw: X = %+5d, Y = %+5d, Z = %+5d\r\n",
				*((int16_t *) &aBuffer[0]),
				*((int16_t *) &aBuffer[2]),
				*((int16_t *) &aBuffer[4]));
	}

	return Ret;
}

uint16_t L3G4200D_GetData(float *pX_rads, float *pY_rads, float *pZ_rads)
{
	uint16_t Ret = 0;
	int16_t aBuffer[3];

	/* Read X, Y and Z in raw */
	Ret = L3G4200D_GetDataRaw(&aBuffer[0], &aBuffer[1], &aBuffer[2]);
	if (Ret == HAL_OK)
	{
		if (pX_rads != NULL)
			*pX_rads = (float) aBuffer[0] * 0.00875 * M_PI / 180.0;

		if (pY_rads != NULL)
			*pY_rads = (float) aBuffer[1] * 0.00875 * M_PI / 180.0;

		if (pZ_rads != NULL)
			*pZ_rads = (float) aBuffer[2] * 0.00875 * M_PI / 180.0;

		LOG_DEBUG("G: X = %+1.3f, Y = %+1.3f, Z = %+1.3f\r\n",
				aBuffer[0] * 0.00875 * M_PI / 180.0,
				aBuffer[1] * 0.00875 * M_PI / 180.0,
				aBuffer[2] * 0.00875 * M_PI / 180.0);
	}

	return Ret;
}
