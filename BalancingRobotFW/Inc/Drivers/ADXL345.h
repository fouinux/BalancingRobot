/*
 * ADXL345.h
 *
 *  Created on: 20 mars 2018
 *      Author: fouinux@gmail.com
 */

#ifndef DRIVERS_ADXL345_H_
#define DRIVERS_ADXL345_H_

/* ADXL345 general definition */
#define ADXL345_I2C_ADDR							0xA6

#define ADXL345_GAIN_FULL_RESOLUTION				((float) 0.0039)
#define ADXL345_GAIN_2G								((float) 0.0039)
#define ADXL345_GAIN_4G								((float) 0.0078)
#define ADXL345_GAIN_8G								((float) 0.0156)
#define ADXL345_GAIN_16G							((float) 0.0322)

#define ADXL345_GAIN_OFFSET							((float) 0.0156)


/* ADXL345 register definition */
#define ADXL345_REG_ADDR_DEVID						0x00
	#define ADXL345_DEVID									0xE5

#define ADXL345_REG_ADDR_THRESH_TAP					0x1D
#define ADXL345_REG_ADDR_OFSX						0x1E
#define ADXL345_REG_ADDR_OFSY						0x1F
#define ADXL345_REG_ADDR_OFSZ						0x20
#define ADXL345_REG_ADDR_DUR						0x21
#define ADXL345_REG_ADDR_LATENT						0x22
#define ADXL345_REG_ADDR_WINDOW						0x23
#define ADXL345_REG_ADDR_THRESH_ACT					0x24
#define ADXL345_REG_ADDR_THRESH_INACT				0x25
#define ADXL345_REG_ADDR_TIME_INACT					0x26
#define ADXL345_REG_ADDR_ACT_INACT_CTL				0x27
#define ADXL345_REG_ADDR_THRESH_FF					0x28
#define ADXL345_REG_ADDR_TIME_FF					0x29
#define ADXL345_REG_ADDR_TAP_AXES					0x2A
#define ADXL345_REG_ADDR_ACT_TAP_STATUS				0x2B

#define ADXL345_REG_ADDR_BW_RATE					0x2C
	#define ADXL345_BW_RATE_BN_LOW_POWER					4
	#define ADXL345_BW_RATE_BM_LOW_POWER					(0x01 << ADXL345_BW_RATE_BN_LOW_POWER)
	#define ADXL345_BW_RATE_BN_RATE							0
	#define ADXL345_BW_RATE_BM_RATE							(0x0F << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_0_10					(0x00 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_0_20					(0x01 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_0_39					(0x02 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_0_78					(0x03 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_1_56					(0x04 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_3_13					(0x05 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_6_25					(0x06 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_12_5					(0x07 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_25						(0x08 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_50						(0x09 << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_100						(0x0A << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_200						(0x0B << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_400						(0x0C << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_800						(0x0D << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_1600					(0x0E << ADXL345_BW_RATE_BN_RATE)
		#define ADXL345_BW_RATE_BM_RATE_3200					(0x0F << ADXL345_BW_RATE_BN_RATE)

#define ADXL345_REG_ADDR_POWER_CTL					0x2D
	#define ADXL345_POWER_CTL_BN_LINK						5
	#define ADXL345_POWER_CTL_BM_LINK						(0x01 << ADXL345_POWER_CTL_BN_LINK)
	#define ADXL345_POWER_CTL_BN_AUTO_SLEEP					4
	#define ADXL345_POWER_CTL_BM_AUTO_SLEEP					(0x01 << ADXL345_POWER_CTL_BN_AUTO_SLEEP)
	#define ADXL345_POWER_CTL_BN_MEASURE					3
	#define ADXL345_POWER_CTL_BM_MEASURE					(0x01 << ADXL345_POWER_CTL_BN_MEASURE)
	#define ADXL345_POWER_CTL_BN_SLEEP						2
	#define ADXL345_POWER_CTL_BM_SLEEP						(0x01 << ADXL345_POWER_CTL_BN_SLEEP)
	#define ADXL345_POWER_CTL_BN_WAKEUP						0
	#define ADXL345_POWER_CTL_BM_WAKEUP						(0x03 << ADXL345_POWER_CTL_BN_WAKEUP)

#define ADXL345_REG_ADDR_INT_ENABLE					0x2E
#define ADXL345_REG_ADDR_INT_MAP					0x2F
#define ADXL345_REG_ADDR_INT_SOURCE					0x30

#define ADXL345_REG_ADDR_DATA_FORMAT				0x31
	#define ADXL345_DATA_FORMAT_BN_SELF_TEST				7
	#define ADXL345_DATA_FORMAT_BM_SELF_TEST				(0x01 << ADXL345_DATA_FORMAT_BN_SELF_TEST)
	#define ADXL345_DATA_FORMAT_BN_SPI						6
	#define ADXL345_DATA_FORMAT_BM_SPI						(0x01 << ADXL345_DATA_FORMAT_BN_SPI)
	#define ADXL345_DATA_FORMAT_BN_INT_INVERT				5
	#define ADXL345_DATA_FORMAT_BM_INT_INVERT				(0x01 << ADXL345_DATA_FORMAT_BN_INT_INVERT)
	#define ADXL345_DATA_FORMAT_BN_FULL_RES					3
	#define ADXL345_DATA_FORMAT_BM_FULL_RES					(0x01 << ADXL345_DATA_FORMAT_BN_FULL_RES)
	#define ADXL345_DATA_FORMAT_BN_JUSTIFY					2
	#define ADXL345_DATA_FORMAT_BM_JUSTIFY					(0x01 << ADXL345_DATA_FORMAT_BN_JUSTIFY)
	#define ADXL345_DATA_FORMAT_BN_RANGE					0
	#define ADXL345_DATA_FORMAT_BM_RANGE					(0x03 << ADXL345_DATA_FORMAT_BN_RANGE)
	#define ADXL345_DATA_FORMAT_BM_RANGE_2G					(0x00 << ADXL345_DATA_FORMAT_BN_RANGE)
	#define ADXL345_DATA_FORMAT_BM_RANGE_4G					(0x01 << ADXL345_DATA_FORMAT_BN_RANGE)
	#define ADXL345_DATA_FORMAT_BM_RANGE_8G					(0x02 << ADXL345_DATA_FORMAT_BN_RANGE)
	#define ADXL345_DATA_FORMAT_BM_RANGE_16G				(0x03 << ADXL345_DATA_FORMAT_BN_RANGE)

#define ADXL345_REG_ADDR_DATAX0						0x32
#define ADXL345_REG_ADDR_DATAX1						0x33
#define ADXL345_REG_ADDR_DATAY0						0x34
#define ADXL345_REG_ADDR_DATAY1						0x35
#define ADXL345_REG_ADDR_DATAZ0						0x36
#define ADXL345_REG_ADDR_DATAZ1						0x37

#define ADXL345_REG_ADDR_FIFO_CTL					0x38
	#define ADXL345_FIFO_CTL_BN_FIFO_MODE				6
	#define ADXL345_FIFO_CTL_BM_FIFO_MODE				(0x03 << ADXL345_FIFO_CTL_BN_FIFO_MODE)
		#define ADXL345_FIFO_CTL_BM_FIFO_MODE_BYPASS		(0x00 << ADXL345_FIFO_CTL_BN_FIFO_MODE)
		#define ADXL345_FIFO_CTL_BM_FIFO_MODE_FIFO			(0x01 << ADXL345_FIFO_CTL_BN_FIFO_MODE)
		#define ADXL345_FIFO_CTL_BM_FIFO_MODE_STREAM		(0x02 << ADXL345_FIFO_CTL_BN_FIFO_MODE)
		#define ADXL345_FIFO_CTL_BM_FIFO_MODE_TRIGGER		(0x03 << ADXL345_FIFO_CTL_BN_FIFO_MODE)
	#define ADXL345_FIFO_CTL_BN_TRIGGER					5
	#define ADXL345_FIFO_CTL_BM_TRIGGER					(0x01 << ADXL345_FIFO_CTL_BN_TRIGGER)
	#define ADXL345_FIFO_CTL_BN_SAMPLE					0
	#define ADXL345_FIFO_CTL_BM_SAMPLE					(0x1F << ADXL345_FIFO_CTL_BN_SAMPLE)

#define ADXL345_REG_ADDR_FIFO_STATUS				0x39

/* Public functions */
uint16_t ADXL345_Init(void);
uint16_t ADXL345_GetDataRaw(int16_t *pX, int16_t *pY, int16_t *pZ);
uint16_t ADXL345_GetData(float *pX_g, float *pY_g, float *pZ_g);
uint16_t ADXL345_OffsetCalibration(void);

#endif /* DRIVERS_ADXL345_H_ */
