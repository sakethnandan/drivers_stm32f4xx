/*
 * 1_led_toggle.c
 *
 *  Created on: Jul 11, 2019
 *      Author: Nandan
 */

#include "stm32f446xx.h"


void delay(void){
	for(uint32_t i = 0; i< 500000; i++);
}

int main(void){
	GPIO_Handle_t gpio_led;
	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNum = 5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClkControl(GPIOA, ENABLE);

	GPIO_Init(&gpio_led);

/*	while(1){
		GPIO_ToggleOutputPin(GPIOA, gpio_led.GPIO_PinConfig.GPIO_PinNum);
		delay();
		GPIO_ToggleOutputPin(GPIOA, gpio_led.GPIO_PinConfig.GPIO_PinNum);
		delay();

	}
*/

	GPIO_WriteToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNum, DISABLE);
	//GPIO_DeInit(GPIOA);

	return 0;
}
