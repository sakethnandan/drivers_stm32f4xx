/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: Jul 2, 2019
 *      Author: Nandan
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"


/*
 * Configuration structure of a GPIO pin
 */

typedef struct{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;		// Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;		// Possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;// Possible values from @GPIO_PIN_PULL_UP_DOWN
	uint8_t GPIO_PinOPType;		// Possible values from @GPIO_PIN_OPTYPE
	uint8_t GPIO_PinAltFun;
}GPIO_PinConfig_t;

/*
 * Handle structure for GPIO pin
 */

typedef struct{
	GPIO_RegDef_t *pGPIOx; /* Holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin - possible modes
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4 // Input falling edge trigger
#define GPIO_MODE_IT_RT 	5 // Input rising edge trigger
#define GPIO_MODE_IT_RFT 	6 // Input rising-falling edge trigger


/*
 * @GPIO_PIN_OPTYPE
 * GPIO possible pin output types
 */

#define GPIO_OP_TYPE_PP		0 //Push-pull
#define GPIO_OP_TYPE_OD		1 //Open Drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin speed
 */

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PULL_UP_DOWN
 * GPIO pin pull up and pull down configuration
 */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2
/*
 * GPIO Driver APIs
 */


/*
 * Peripheral Clock setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Initialize and De-initialize
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); // Make use of the peripheral reset register of RCC to reset the GPIO

/*
 * Data Read and Write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);



/*
 * Interrupt Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t PinNum);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
