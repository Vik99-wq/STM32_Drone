#include "esc.h"
#include "stm32f4xx_hal_tim.h"

//https://dronenodes.com/wp-content/uploads/2018/10/quadcopter-motor-cw-ccw.jpg

void setSpeeds(TIM_HandleTypeDef * htim2, uint8_t esc1, uint8_t esc2, uint8_t esc3, uint8_t esc4){

	// start tim2 for pwm
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);

	// put the pwm signal into each tim2's register
	htim2->Instance->CCR1 = esc1;
	htim2->Instance->CCR2 = esc2;
	htim2->Instance->CCR3 = esc3;
	htim2->Instance->CCR4 = esc4;

}

// get the current tim2 pwm signal strengths
int getPwm(TIM_HandleTypeDef * htim2, const char *esc) {

	if (strcmp(esc, "esc1") == 0)
		return htim2->Instance->CCR1;
	else if (strcmp(esc, "esc2") == 0)
		return htim2->Instance->CCR2;
	else if (strcmp(esc, "esc3") == 0)
		return htim2->Instance->CCR3;
	else if (strcmp(esc, "esc4") == 0)
		return htim2->Instance->CCR4;

	return 1;

}

void setThrottle(TIM_HandleTypeDef * htim2, uint8_t throttle){

	setSpeeds(htim2, throttle, throttle, throttle, throttle);

}

void tim2Init(TIM_HandleTypeDef *htim2){

	HAL_TIM_PWM_Init(htim2);

}
