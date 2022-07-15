#include "main.h"

uint16_t accelData[3];
uint16_t gyroData[3];
uint8_t raw_data[12];
uint8_t regs[12];
uint8_t i;
uint8_t WHO_AM_I;

// for each loop macro
#define foreach(item, array) \
    for(int keep = 1, \
            count = 0,\
            size = sizeof (array) / sizeof *(array); \
        keep && count != size; \
        keep = !keep, count++) \
      for(item = (array) + count; keep; keep = !keep)
