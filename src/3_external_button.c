/*
 * 3_external_button.c
 *
 *  Created on: Jul 16, 2019
 *      Author: Nandan
 */

#include "stm32f446xx.h"


/*
 * We connect the circuit in such a way
 * as when the button is pressed the pin is
 * pulled low
 * Refer to the circuit diagram for more info
 */
#define BTN_PRSD	0

void delay(void){
	for(uint32_t i = 0; i< 500000; i++);
}

int main(void){
	GPIO_Handle_t ext_led, ext_btn;
	ext_led.pGPIOx = GPIOB;
	ext_led.GPIO_PinConfig.GPIO_PinNum = 5;
	ext_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	ext_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ext_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	ext_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClkControl(ext_led.pGPIOx, ENABLE);
	GPIO_Init(&ext_led);

	ext_btn.pGPIOx = GPIOA;
	ext_btn.GPIO_PinConfig.GPIO_PinNum = 10;
	ext_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	ext_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	ext_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	GPIO_PeriClkControl(ext_btn.pGPIOx, ENABLE);
	GPIO_Init(&ext_btn);



	while(1){
		if(GPIO_ReadFromInputPin(ext_btn.pGPIOx, ext_btn.GPIO_PinConfig.GPIO_PinNum) == BTN_PRSD){
			delay();
			GPIO_ToggleOutputPin(ext_led.pGPIOx, ext_led.GPIO_PinConfig.GPIO_PinNum);
		}


	}


	//GPIO_WriteToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNum, DISABLE);
	//GPIO_DeInit(GPIOA);

	return 0;
}
