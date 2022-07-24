#include <math.h>
#include <stdlib.h>
#include "imu.h"
#include "spi.h"
#include "pid.h"

double phiHat_rad = 0.0;
double thetaHat_rad = 0.0;
PID_TypeDef d_PID;
double phi_PIDOut;
double theta_PIDOut;
double phiSetpoint = 0;
double thetaSetpoint = 0;
uint16_t accelData[3];
uint16_t gyroData[3];
uint8_t accel_isNegative[3];
uint8_t gyro_isNegative[3];
uint8_t raw_data[12];
uint8_t WHO_AM_I;
uint8_t accel_firstDigit;
uint8_t gyro_firstDigit;
char accelData_temp[15];
char gyroData_temp[15];
int16_t signed_accelData[3];
int16_t signed_gyroData[3];

extern double phiHat_deg;
extern double thetaHat_deg;

// accel XYZ then gyro XZY, high register then low register
uint8_t regs[] = {0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};

uint8_t gyro_Sensitivity = 65.5;
uint16_t accel_Sensitivity = 4096;

int16_t binaryTodecimal(uint16_t binary);
uint8_t IAM_writeBytes(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t data);
uint8_t IAM_readBytes(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t *data);
uint8_t IAM_read_AccelGyro_Bytes(SPI_HandleTypeDef *hspi1, uint8_t* regs, uint8_t *data);
void IAM_CSHigh();
void IAM_CSLow();
void signData(uint16_t* tempData, uint16_t* oldData, int16_t* signedData, uint8_t firstDigit, uint8_t i, uint8_t* isNegitive);


uint8_t IAM_read_AccelGyro_Bytes(SPI_HandleTypeDef *hspi1, uint8_t* regs, uint8_t *data)
{
	uint8_t status;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];

	for(uint8_t i = 0; i < 12; i++)
	{
		txBuf[0] = (*(regs + i) | 0x80);
		txBuf[1] = 0x00;

		IAM_CSLow();

		// data transmit
		status = (HAL_SPI_TransmitReceive(hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);

		IAM_CSHigh();
		// save data to given pointer
		if (status == 1)
		{
			*(data + i) = rxBuf[1];
		}
	}

	return status;
}

uint8_t IAM_readBytes(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t *data)
{
	uint8_t status;
	uint8_t rxBuf[2];
	uint8_t txBuf[2];

	txBuf[0] = (addr | 0x80);
	txBuf[1] = 0x00;

	IAM_CSLow();

	// data transmit
	status = (HAL_SPI_TransmitReceive(hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);

	IAM_CSHigh();
	// save data to given pointer
	if (status == 1)
	{
		*(data) = rxBuf[1];
	}

	return status;

}

uint16_t * IAM_readAccelGyro(SPI_HandleTypeDef *hspi1)
{
	IAM_read_AccelGyro_Bytes(hspi1, regs, &raw_data);
	// goes XYZ

	// add the low and high byte to the accel variables
	accelData[0] = (uint16_t) (raw_data[0]<<8 | raw_data[1]);
	accelData[1] = (uint16_t) (raw_data[2]<<8 | raw_data[3]);
	accelData[2] = (uint16_t) (raw_data[4]<<8 | raw_data[5]);

	// add the low and high byte to the gyro variables
	gyroData[0] = (uint16_t) (raw_data[6]<<8 | raw_data[7]);
	gyroData[1] = (uint16_t) (raw_data[8]<<8 | raw_data[9]);
	gyroData[2] = (uint16_t) (raw_data[10]<<8 | raw_data[11]);

	for (int i = 0; i < 3; i++)
	{
		// checking accel data sign
		signData(&accelData_temp[i], &accelData[i], &signed_accelData[i], accel_firstDigit, i, accel_isNegative);
		// convert data to decimal and correct signing
		if (accel_isNegative[i] == 1)
		{
			signed_accelData[i] = (binaryTodecimal(*(signed_accelData + i)) * -1);
		}
		else
		{
			signed_accelData[i] = binaryTodecimal(*(signed_accelData + i));
		}

		// checking gyro data sign
		signData(&gyroData_temp[i], &gyroData[i], &signed_gyroData[i], gyro_firstDigit, i, gyro_isNegative);
		// convert data to decimal and correct signing
		if (gyro_isNegative[i] == 1)
		{
			signed_gyroData[i] = (binaryTodecimal(*(signed_gyroData + i)) * -1);
		}
		else
		{
			signed_gyroData[i] = binaryTodecimal(*(signed_gyroData + i));
		}
	}

	// gyro_Sensitivity = 65.5 LSB for 500dps config
	// converts the RAW values into dps
	signed_gyroData[0] /= gyro_Sensitivity;
	signed_gyroData[1] /= gyro_Sensitivity;
	signed_gyroData[2] /= gyro_Sensitivity;

	// accel_Sensitivity = 4096 LSB/g for 8g config
	// converts the raw accel data into g
	signed_accelData[0] /= accel_Sensitivity;
	signed_accelData[1] /= accel_Sensitivity;
	signed_accelData[2] /= accel_Sensitivity;

	uint16_t retVal[2];
	retVal[0] = signed_accelData;
	retVal[1] = signed_gyroData;

	return retVal;
}

uint8_t IAM_writeBytes(SPI_HandleTypeDef *hspi1, uint8_t addr, uint8_t data)
{
	uint8_t txBuf[2] = {addr, data};

	IAM_CSLow();

	// data transmit
	uint8_t status = (HAL_SPI_Transmit(hspi1, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);

	IAM_CSHigh();

	return status;
}

void IAM_Init (SPI_HandleTypeDef *hspi1)
{
	// set the PWR_MGMT_1 register (6B hex) bits as 0x01 to activate the gyro
	IAM_writeBytes(hspi1, 0x6B, 0x01);
	HAL_Delay(10);

	// set the SMPLRT_DIV register (19 hex) bits as 0x09 to divide sample rate by 1khz to 100hz
	IAM_writeBytes(hspi1, 0x19, 0x09);
	HAL_Delay(10);

	// set the GYRO_CONFIG register (1B hex) bits as 00001000 (500dps full scale)
	IAM_writeBytes(hspi1, 0x1B, 0b00001000);
	HAL_Delay(10);

	// set the ACCEL_CONFIG register (1C hex) bits as 00010000 (8g full scale)
	IAM_writeBytes(hspi1, 0x1c, 0b00010000);
	HAL_Delay(10);
}

// pull CS high
void IAM_CSHigh()
{
	HAL_GPIO_WritePin(GPIOA, IMU_NCS_Pin, GPIO_PIN_SET);
}

// pull CS low
void IAM_CSLow()
{
	HAL_GPIO_WritePin(GPIOA, IMU_NCS_Pin, GPIO_PIN_RESET);
}

// IAM Identifier
uint8_t IAM_WHOAMI(SPI_HandleTypeDef *hspi1)
{
	// read the WHO_AM_I register (75 hex) byte to access the 8-bit device ID
	uint8_t status = IAM_readBytes(hspi1, 0x75, &WHO_AM_I);

	return status;
}

// Transform body angles to euler angles referenced to the earth
void complemetaryFilter(uint16_t* accelData, uint16_t* gyroData)
{
	// Estimate angles using accelerometer measurements
	double phiHat_acc_rad = atanf(*(accelData + 1) / *(accelData + 2));
	double thetaHat_acc_rad = asinf(*accelData / g);

	double phiDot_rps = *gyroData + tanf(thetaHat_rad) * (sinf(phiHat_rad) * (*(gyroData + 1)) + cosf(phiHat_rad) * (*(gyroData + 2)));
	double thetaDot_rps = cosf(phiHat_rad) * gyroData[1] - sinf(phiHat_rad) * (*(gyroData + 2));

	// mesh gyro angles with accelerometer readings to counteract gyro drift
	phiHat_rad = COMP_FILT_ALPHA * phiHat_acc_rad + (1.0f - COMP_FILT_ALPHA) * (phiHat_rad + (SAMPLE_TIME_MS_USB / 1000.0f) * phiDot_rps);
	thetaHat_rad = COMP_FILT_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FILT_ALPHA) * (thetaHat_rad + (SAMPLE_TIME_MS_USB / 1000.0f) * thetaDot_rps);

	// convert phi hat and rheta Hat from radians to degrees
	phiHat_deg = phiHat_rad * RAD_TO_DEG;
	thetaHat_deg = thetaHat_rad * RAD_TO_DEG;
}

int16_t binaryTodecimal(uint16_t binary)
{
	int16_t dec = 0;
	uint8_t i = 0;
	uint8_t rem;

	while (binary!=0)
	{
	  rem = binary % 10;
	  binary /= 10;
	  dec += rem * pow(2, i);
	  ++i;
	}

	return dec;
}

void signData(uint16_t* tempData, uint16_t* oldData, int16_t* signedData, uint8_t firstDigit, uint8_t i, uint8_t* isNegitive)
{
	firstDigit = (int8_t)(*(oldData + i) / pow(10, 15));

	if (firstDigit == 0)
	{
		for (int z = 0; z < 14; z++)
		{
			*(tempData + z) = *(oldData + z + 1);
		}
		*(signedData + i) = atoi(*tempData);
	}
	else
	{
		*(isNegitive + i) = 1;
	}
}

void Phi_pid_init()
{
	// initialize the PID
	PID(&d_PID, &phiHat_deg, &phi_PIDOut, &phiSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	// set to auto mode
	PID_SetMode(&d_PID, _PID_MODE_AUTOMATIC);
	// PID is being run every 30ms
	PID_SetSampleTime(&d_PID, 30);
	// outputs must be -180deg to 180deg
	PID_SetOutputLimits(&d_PID, -180, 180);
}

int32_t Phi_pid_run(void)
{
	// run the phi PID iteration
	PID_Compute(&d_PID);
	HAL_Delay(10);

	return phi_PIDOut;
}

void Theta_pid_init()
{
	// initialize the PID
	PID(&d_PID, &thetaHat_deg, &theta_PIDOut, &thetaSetpoint, 2, 5, 1, _PID_P_ON_E, _PID_CD_DIRECT);
	// set to auto mode
	PID_SetMode(&d_PID, _PID_MODE_AUTOMATIC);
	// PID is being run every 30ms
	PID_SetSampleTime(&d_PID, 30);
	// outputs must be -180deg to 180deg
	PID_SetOutputLimits(&d_PID, -180, 180);
}

int32_t Theta_pid_run(void)
{
	// run the theta PID iteration
	PID_Compute(&d_PID);
	HAL_Delay(10);

	return theta_PIDOut;
}
