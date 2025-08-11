#include "BMI088.h"
#include <math.h>

extern float roll, pitch,yaw;
uint32_t lastTime = 0;
float gyro_z_offset = 0.0f;
/*
 *
 * INITIALISATION
 *
 */

uint8_t BMI088_Init_I2C(BMI088 *imu,
                        I2C_HandleTypeDef *i2cHandle,
                        uint8_t accAddress, uint8_t gyrAddress) {

    imu->i2cHandle = i2cHandle;
    imu->accAddress = 0x18 << 1;  // = 0x30
    imu->gyrAddress = 0x68 << 1;  // = 0xD0

    imu->readingAcc = 0;
    imu->readingGyr = 0;

    uint8_t status = 0;
    uint8_t data = 0;

    if (HAL_I2C_IsDeviceReady(i2cHandle, 0x68 << 1, 3, 100) == HAL_OK) {
        // BMI bulundu
    	printf("ok");
    } else {
        // BMI bağlı değil / bus kitli
    }

    if (HAL_I2C_IsDeviceReady(i2cHandle, imu->accAddress, 3, 100) == HAL_OK)
 {
        // devam et
    }
    else {
        // adres yok, hata var
    }

    /*
     *
     * ACCELEROMETER
     *
     */

    // Soft reset accelerometer
    data = 0xB6;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_ACC_SOFTRESET, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(50);

    // Read and check chip ID
    status += HAL_I2C_Mem_Read(i2cHandle, imu->accAddress, BMI_ACC_CHIP_ID, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x1E) {
        return 1; // ACC chip ID mismatch
    }

    HAL_Delay(10);

    // Configure accelerometer
    data = 0xA8;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_ACC_CONF, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x00; // ±3g
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_ACC_RANGE, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Enable accelerometer interrupts (optional)
    data = 0x0A;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_INT1_IO_CONF, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x04;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_INT1_INT2_MAP_DATA, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Power ON
    data = 0x00;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_ACC_PWR_CONF, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x04;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->accAddress, BMI_ACC_PWR_CTRL, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f;
    imu->accTxBuf[0] = BMI_ACC_DATA;

    /*
     *
     * GYROSCOPE
     *
     */

    // Soft reset gyro
    data = 0xB6;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_GYR_SOFTRESET, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(250);

    // Read and check chip ID
    status += HAL_I2C_Mem_Read(i2cHandle, imu->gyrAddress, BMI_GYR_CHIP_ID, 1, &data, 1, HAL_MAX_DELAY);
    if (data != 0x0F) {
        return 2; // GYR chip ID mismatch
    }

    HAL_Delay(10);

    // Configure gyroscope
    data = 0x01; // ±1000 dps
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_GYR_RANGE, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x07;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_GYR_BANDWIDTH, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Gyro interrupts (optional)
    data = 0x80;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_GYR_INT_CTRL, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x01;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_INT3_INT4_IO_CONF, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    data = 0x01;
    status += HAL_I2C_Mem_Write(i2cHandle, imu->gyrAddress, BMI_INT3_INT4_IO_MAP, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f;
    imu->gyrTxBuf[0] = BMI_GYR_DATA;

    return status;
}

uint8_t rollpitchyaw(BMI088 *imu){
           // Derece cinsinden yön açısı



	    uint32_t now = HAL_GetTick();  // ms
	    float dt = (now - lastTime) / 1000.0f;
	    lastTime = now;

	    // İvme verilerini oku
	    float acc_x = imu->acc_mps2[0];
	    float acc_y = imu->acc_mps2[1];
	    float acc_z = imu->acc_mps2[2];

	    // Roll ve pitch hesapla (ivme verisi ile)
	    roll  = atan2f(acc_y, acc_z) * 180.0f / 3.14159f;
	  	pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0f / 3.14159f;

	    // Yaw hesapla (gyro verisi ile)
	    float gyro_z_deg = (imu->gyr_rps[2] - gyro_z_offset) * 180.0f / 3.14159f;
	    yaw += gyro_z_deg * dt;

	    // Yaw açısını -180 ~ +180 arasında tut
	    if (yaw > 180.0f) yaw -= 360.0f;
	    if (yaw < -180.0f) yaw += 360.0f;

	    return 1;
}
/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
	return (HAL_I2C_Mem_Read(imu->i2cHandle, imu->accAddress, regAddr, 1, data, 1, HAL_MAX_DELAY) == HAL_OK);
}


uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
	return (HAL_I2C_Mem_Read(imu->i2cHandle, imu->gyrAddress, regAddr, 1, data, 1, HAL_MAX_DELAY) == HAL_OK);
}


uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
	return (HAL_I2C_Mem_Write(imu->i2cHandle, imu->accAddress, regAddr, 1, &data, 1, HAL_MAX_DELAY) == HAL_OK);
}


uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
	return (HAL_I2C_Mem_Write(imu->i2cHandle, imu->gyrAddress, regAddr, 1, &data, 1, HAL_MAX_DELAY) == HAL_OK);
}




/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu) {
	uint8_t acc_raw[6];
		  int16_t acc_x_raw, acc_y_raw, acc_z_raw;
		  float accel_range_factor; // ACC_RANGE register'ına göre hesaplanacak

		  // 0x12'den 6 byte oku (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
		  HAL_I2C_Mem_Read(imu->i2cHandle, imu->accAddress, 0x12, 1, acc_raw, 6, 100);

		  // RAW verileri 16-bit signed sayıya dönüştür
		  acc_x_raw = (int16_t)((acc_raw[1] << 8) | acc_raw[0]);
		  acc_y_raw = (int16_t)((acc_raw[3] << 8) | acc_raw[2]);
		  acc_z_raw = (int16_t)((acc_raw[5] << 8) | acc_raw[4]);

		  // ACC_RANGE register (0x41) oku
		  uint8_t acc_range_raw;
		  HAL_I2C_Mem_Read(imu->i2cHandle, imu->accAddress, 0x41, 1, &acc_range_raw, 1, 100);

		  // range: 0 → ±3g, 1 → ±6g, 2 → ±12g, 3 → ±24g gibi olabilir (datasheet'e göre)
		  accel_range_factor = powf(2, (acc_range_raw + 1)) * 1.5f;  // g cinsinden

		  // mg → m/s² dönüşüm (1g = 9.80665 m/s²)
		  imu->acc_mps2[0] = ((float)acc_x_raw / 32768.0f) * accel_range_factor * 9.80665f;
		  imu->acc_mps2[1] = ((float)acc_y_raw / 32768.0f) * accel_range_factor * 9.80665f;
		  imu->acc_mps2[2] = ((float)acc_z_raw / 32768.0f) * accel_range_factor * 9.80665f;

}



uint8_t BMI088_ReadGyroscope(BMI088 *imu) {
	 uint8_t gyro_raw[6];
	    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
	    float gyro_sensitivity = 2000.0f / 32768.0f; // default range: ±2000 dps

	    // 0x02'den başlayarak 6 byte oku (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
	    if (HAL_I2C_Mem_Read(imu->i2cHandle, imu->gyrAddress, 0x02, 1, gyro_raw, 6, 100) != HAL_OK) {
	        return 1; // Hata
	    }

	    // 16-bit signed dönüşüm (MSB önce shiftlenir)
	    gyro_x_raw = (int16_t)((gyro_raw[1] << 8) | gyro_raw[0]);
	    gyro_y_raw = (int16_t)((gyro_raw[3] << 8) | gyro_raw[2]);
	    gyro_z_raw = (int16_t)((gyro_raw[5] << 8) | gyro_raw[4]);
	    // Açısal hızları hesapla (dps cinsinden)
	        imu->gyr_dps[0] = (float)gyro_x_raw * gyro_sensitivity;
	        imu->gyr_dps[1] = (float)gyro_y_raw * gyro_sensitivity;
	        imu->gyr_dps[2] = (float)gyro_z_raw * gyro_sensitivity;

	        // Eğer rad/s istiyorsan ayrıca şunu da hesaplayabilirsin:
	        float deg2rad = 3.14159f / 180.0f;
	        imu->gyr_rps[0] = imu->gyr_dps[0] * deg2rad;
	        imu->gyr_rps[1] = imu->gyr_dps[1] * deg2rad;
	        imu->gyr_rps[2] = imu->gyr_dps[2] * deg2rad;

	        return 0;
}


/*
 *
 * DMA
 *
 */
uint8_t BMI088_ReadAccelerometerDMA(BMI088 *imu) {
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, imu->accAddress, BMI_ACC_DATA, 1,
	                         (uint8_t *)imu->accRxBuf, 6) == HAL_OK) {
		imu->readingAcc = 1;
		return 1;
	} else {
		return 0;
	}
}


void BMI088_ReadAccelerometerDMA_Complete(BMI088 *imu) {
	imu->readingAcc = 0;

	int16_t accX = (int16_t)((imu->accRxBuf[1] << 8) | imu->accRxBuf[0]);
	int16_t accY = (int16_t)((imu->accRxBuf[3] << 8) | imu->accRxBuf[2]);
	int16_t accZ = (int16_t)((imu->accRxBuf[5] << 8) | imu->accRxBuf[4]);

	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;
}


uint8_t BMI088_ReadGyroscopeDMA(BMI088 *imu) {
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, imu->gyrAddress, BMI_GYR_DATA, 1,
	                         (uint8_t *)imu->gyrRxBuf, 6) == HAL_OK) {
		imu->readingGyr = 1;
		return 1;
	} else {
		return 0;
	}
}


void BMI088_ReadGyroscopeDMA_Complete(BMI088 *imu) {
	imu->readingGyr = 0;

	int16_t gyrX = (int16_t)((imu->gyrRxBuf[1] << 8) | imu->gyrRxBuf[0]);
	int16_t gyrY = (int16_t)((imu->gyrRxBuf[3] << 8) | imu->gyrRxBuf[2]);
	int16_t gyrZ = (int16_t)((imu->gyrRxBuf[5] << 8) | imu->gyrRxBuf[4]);

	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;
}

