#ifndef __ESC_H__
#define __ESC_H__

#include "main.h"

void setSpeeds(TIM_HandleTypeDef * htim2, uint8_t esc1, uint8_t esc2, uint8_t esc3, uint8_t esc4);
int getPwm(TIM_HandleTypeDef * htim2, const char *esc);
void setThrottle(TIM_HandleTypeDef * htim2, uint8_t throttle);
void tim2Init(TIM_HandleTypeDef *htim2);

#endif /* __ESC_H__ */
