#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

#define SAMPLE_TIME_MS_USB 20
#define g 9.8100000000f
#define RAD_TO_DEG 57.2957795131f
#define COMP_FILT_ALPHA 0.0500000000f


uint16_t * IAM_readAccelGyro(SPI_HandleTypeDef *hspi1);
void IAM_Init (SPI_HandleTypeDef *hspi1);

uint8_t IAM_WHOAMI(SPI_HandleTypeDef *hspi1);
void complemetaryFilter(uint16_t* accelData, uint16_t* gyroData);

void Phi_pid_init();
int32_t Phi_pid_run(void);
void Theta_pid_init();
int32_t Theta_pid_run(void);

#endif /* __IMU_H__ */
