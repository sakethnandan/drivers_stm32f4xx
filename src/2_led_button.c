/*
 * 2_led_button.c
 *
 *  Created on: Jul 16, 2019
 *      Author: Nandan
 */


#include "stm32f446xx.h"


void delay(void){
	for(uint32_t i = 0; i< 500000/2; i++);
}

int main(void){
	GPIO_Handle_t gpio_led, user_btn;
	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNum = 5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);

	user_btn.pGPIOx = GPIOC;
	user_btn.GPIO_PinConfig.GPIO_PinNum = 13;
	user_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	user_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	user_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClkControl(GPIOC, ENABLE);
	GPIO_Init(&user_btn);



	while(1){
		if(!GPIO_ReadFromInputPin(user_btn.pGPIOx, user_btn.GPIO_PinConfig.GPIO_PinNum)){
			delay();
			GPIO_ToggleOutputPin(GPIOA, gpio_led.GPIO_PinConfig.GPIO_PinNum);
		}


	}


	//GPIO_WriteToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNum, DISABLE);
	//GPIO_DeInit(GPIOA);

	return 0;
}
