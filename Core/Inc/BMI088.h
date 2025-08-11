#ifndef BMI088_IMU_H
#define BMI088_IMU_H

#include "stm32f4xx_hal.h"

/* Register defines */
#define BMI_ACC_CHIP_ID 		0x00
#define BMI_ACC_DATA 			0x12
#define BMI_TEMP_DATA 			0x22
#define BMI_ACC_CONF 			0x40
#define BMI_ACC_RANGE 			0x41
#define BMI_INT1_IO_CONF 	   	0x53
#define BMI_INT1_INT2_MAP_DATA 	0x58
#define BMI_ACC_PWR_CONF 		0x7C
#define BMI_ACC_PWR_CTRL 		0x7D
#define BMI_ACC_SOFTRESET 		0x7E

#define BMI_GYR_CHIP_ID			0x00
#define BMI_GYR_DATA			0x02
#define	BMI_GYR_RANGE			0x0F
#define	BMI_GYR_BANDWIDTH		0x10
#define	BMI_GYR_SOFTRESET		0x14
#define	BMI_GYR_INT_CTRL		0x15
#define	BMI_INT3_INT4_IO_CONF	0x16
#define BMI_INT3_INT4_IO_MAP	0x18

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	uint8_t accTxBuf[1];      // Gerekli değil bile ama uyumluluk için kalabilir
	uint8_t accRxBuf[6];      // 6 byte veri: X, Y, Z (her biri 2 byte)

	uint8_t gyrTxBuf[1];      // Gerekli değil bile ama uyumluluk için kalabilir
	uint8_t gyrRxBuf[6];      // 6 byte veri: X, Y, Z (her biri 2 byte)

	float acc_mps2[3];
	float gyr_rps[3];

	float gyr_dps[3];  // X, Y, Z (derece/saniye)

	float accConversion;
	float gyrConversion;

	uint8_t accAddress;
	uint8_t gyrAddress;

	uint8_t readingAcc;
	uint8_t readingGyr;
} BMI088;


/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init_I2C(BMI088 *imu,
                        I2C_HandleTypeDef *i2cHandle,
                        uint8_t accAddress, uint8_t gyrAddress);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);
uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data);

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);
uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data);

/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu);
uint8_t BMI088_ReadGyroscope(BMI088 *imu);

/*
 *
 * DMA
 *
 */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu);
void 	BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu);

uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu);
void 	BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu);

#endif
