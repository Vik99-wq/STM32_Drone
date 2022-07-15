#include "imu.h"
#include "spi.c"

uint8_t IAM_readBytes(uint8_t regs, uint8_t *data) {
	i = 0;
	foreach(int *reg, regs)
		uint8_t txBuf[2] = {(reg | 0x80), 0x00};
		uint8_t rxBuf[2];

		// pull CS low
		IAM_CSLow();

		// data transmit
		uint8_t status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);

		// pull CS high
		IAM_CSHigh();
		// save data to given pointer
		*data[i] = rxBuf[1];
		i++;

	// 0 if error, 1 if good
	return status;

}

// Accel XYZ then Gyro XZY, high register then low register
regs = {0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};

void IAM_readAccelGyro(void){
	IAM_readBytes(regs, &raw_data);
	// goes XYZ

	// Add the low and high byte to the accel variables
	accelData[0] = (uint16_t) (raw_data[0]<<8 | raw_data[1]);
	accelData[1] = (uint16_t) (raw_data[2]<<8 | raw_data[3]);
	accelData[2] = (uint16_t) (raw_data[4]<<8 | raw_data[5]);

	// Add the low and high byte to the gyro variables
	gyroData[0] = (uint16_t) (raw_data[6]<<8 | raw_data[7]);
	gyroData[1] = (uint16_t) (raw_data[8]<<8 | raw_data[9]);
	gyroData[2] = (uint16_t) (raw_data[10]<<8 | raw_data[11]);

	// Divide or multiply by sensitivity?
	// Divide by sensitivity

	// Gyro_Sensitivity = 65.5 LSB for 500dps config
	// converts the RAW values into dps

	gyroData[0] /= 65.5;
	gyroData[1] /= 65.5;
	gyroData[2] /= 65.5;

	// Accel_Sensitivity = 4096 LSB/g for 8g config
	// converts the raw accel data into g
	accelData[0] /= 4096;
	accelData[1] /= 4096;
	accelData[2] /= 4096;
}

uint8_t IAM_writeBytes(uint8_t addr, uint8_t data){

	uint8_t txBuf[2] = {addr, data};

	// pull CS low
	IAM_CSLow();

	// data transmit
	uint8_t status = (HAL_SPI_Transmit(&hspi1, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);

	// pull CS high
	IAM_CSHigh();

	return status;
}

void IAM_Initialize (void){
	// Set the PWR_MGMT_1 register (6B hex) bits as 0x01 to activate the gyro
	IAM_writeBytes(0x6B, 0x01);

	// Set the SMPLRT_DIV register (19 hex) bits as 0x09 to divide sample rate by 1khz to 100hz
	IAM_writeBytes(0x19, 0x09);

	// Set the GYRO_CONFIG register (1B hex) bits as 00001000 (500dps full scale)
	IAM_writeBytes(0x1B, 0b00001000);

	// Set the ACCEL_CONFIG register (1C hex) bits as 00010000 (8g full scale)
	IAM_writeBytes(0x1c, 0b00010000);
}

void IAM_CSHigh(void) {
	HAL_GPIO_WritePin(GPIOA, SPI1_IMU_NCS_Pin, GPIO_PIN_SET);
}

void IAM_CSLow(void) {
	HAL_GPIO_WritePin(GPIOA, SPI1_IMU_NCS_Pin, GPIO_PIN_RESET);
}

uint8_t IAM_WHOAMI(void) {
	// Read the WHO_AM_I register (75 hex) byte to access the 8-bit device ID
	IAM_readBytes(0x75, &WHO_AM_I);
}
