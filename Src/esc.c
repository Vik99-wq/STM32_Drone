#include "esc.h"
#include "imu.c"

void setSpeeds(uint8_t esc1, uint8_t esc2, uint8_t esc3, uint8_t esc4){

	// start tim2 for pwm
	HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_4);

	// Is pwm signal inverted?

	// put the pwm signal into each tim2's register
	htim2.Instance->CCR1 = 200 - esc1;
	htim2.Instance->CCR2 = 200 - esc2;
	htim2.Instance->CCR3 = 200 - esc3;
	htim2.Instance->CCR4 = 200 - esc4;
}

// get the current tim2 pwm signal strengths
void getPwm(char esc){
	switch(esc){
		case "esc1":
			return htim2.Instance->CCR1;
		case "esc2":
			return htim2.Instance->CCR2;
		case "esc3":
			return htim2.Instance->CCR3;
		case "esc4":
			return htim2.Instance->CCR4;
		default:
			return 0;
	}
}


