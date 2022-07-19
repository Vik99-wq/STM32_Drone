#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

#define SAMPLE_TIME_MS_USB 20
#define g 9.8100000000f
#define RAD_TO_DEG 57.2957795131f
#define COMP_FILT_ALPHA 0.0500000000f

double phiHat_deg = 0.0;
double thetaHat_deg = 0.0;

#endif /* __IMU_H__ */
