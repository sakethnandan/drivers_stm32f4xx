/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Jul 2, 2019
 *      Author: Nandan
 */
#include "stm32f446xx_gpio_driver.h"

/**
 * This function enables or disables the peripheral clock for given GPIO port
 * @param pGPIO	- Base address of the gpio peripheral
 * @param EnorDi - Enable or Disable macros
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}

	}
}

/**
 * Initialize the GPIO port with the mode, speed, pull-up/pull-down, output type and alternate
 * functionality configuration
 *
 * @param pGPIOHandle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	// 1. Configure the mode of the GPIO pin
	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); //Clear the bit-fields
		//pGPIOHandle->pGPIOx->MODER &= ~(1 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); //Clear the bit-fields
		pGPIOHandle->pGPIOx->MODER = temp;


	}else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			// Clear the RTSR register of that pin
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);
		//3. Enable the EXTI interrupt delivery using Interrupt Mask Register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);

	}
	temp = 0;
	//2. Configure the speed of the pin
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); //Clear the bit-fields
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//3. Pull-up/down configuration
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum)); //Clear the bit-fields
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNum); //Clear the bit-fields
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alternate functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFun == GPIO_MODE_ALTFN){
		//Pin 0-7 uses the AFRL(AFR[0]) register and 8-15 uses the AFRH(AFR[1]) register
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 8; // temp1 will decide which register (AFRL or AFRH) to be used
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 8; //  temp2 will decide the position of the bits to be shifted
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2)); //Clear the bit-fields
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFun << (4*temp2));
	}
}
/**
 * Resets the GPIO to its initial state. This is done using the peripheral reset register
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
			GPIOA_RST_REG();
			}else if(pGPIOx == GPIOB){
				GPIOB_RST_REG();
			}else if(pGPIOx == GPIOC){
				GPIOC_RST_REG();
			}else if(pGPIOx == GPIOD){
				GPIOD_RST_REG();
			}else if(pGPIOx == GPIOE){
				GPIOE_RST_REG();
			}else if(pGPIOx == GPIOF){
				GPIOF_RST_REG();
			}else if(pGPIOx == GPIOG){
				GPIOG_RST_REG();
			}else if(pGPIOx == GPIOH){
				GPIOH_RST_REG();
			}


}

/*
 * Data Read and Write
 */


/**
 * Check the status of a particular pin of the given port
 * @param pGPIOx
 * @param PinNum
 * @return 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNum) & (0x00000001));
	return value;
}

/**
 * Check the status of the entire port
 * @param pGPIOx
 * @return
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
/**
 * Write a value to a pin - Set/Unset
 * @param pGPIOx - A - H
 * @param PinNum - 0 - 15
 * @param Value - 0 or 1
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Value){
	if(Value == ENABLE){
		pGPIOx->ODR |= (1 << PinNum);
	}
	else if(Value == DISABLE){
		pGPIOx->ODR &= ~(1 << PinNum);
	}else{

	}

}

/**
 * Write the entire value to a port
 * @param pGPIOx
 * @param Value
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

/**
 *
 * @param pGPIOx
 * @param PinNum
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum){
	pGPIOx->ODR ^= (1 << PinNum);
}



/*
 * Interrupt Handling
 * 1. Pin must be in input configuration
 * 2. Configure the edge trigger (RT, FT, RFT)
 * 3. Enable interrupt delivery from peripheral to the processor (on peripheral side)
 * 4. Identify the IRQ number on which the processor accepts the interrupt from that pin
 * 5. Configure the IRQ priority for the identified IRQ number (Processor side)
 * 6. Enable interrupt reception on that IRQ number (Processor side)
 * 7. Implement IRQ Handler
 */

/**
 *
 * @param IRQNumber
 * @param EnorDi
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			// Program Interrupt Set Enable Register 0 (ISER0) of NVIC controller
			*NVIC_ISER0 |= (1 << IRQNumber) ;

		}else if(IRQNumber > 31 && IRQNumber <64){
			*NVIC_ISER1 |= (1 << IRQNumber%32) ;

		}else if(IRQNumber >= 64 && IRQNumber <96){
			*NVIC_ISER2 |= (1 << IRQNumber%64) ;

		}else if(IRQNumber >= 96 && IRQNumber <128){


		}else if(IRQNumber >= 128 && IRQNumber <160){

		}else if(IRQNumber >= 160 && IRQNumber <192){

		}else if(IRQNumber >= 192 && IRQNumber <224){

		}else if(IRQNumber >= 224 && IRQNumber <256){

		}
	}
	else{
		if(IRQNumber <= 31){
			// Program Interrupt Clear Enable Register 0 (ICER0) of NVIC controller
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber <64){
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber <96){
			*NVIC_ICER2 |= (1 << IRQNumber % 64);

		}else if(IRQNumber >= 96 && IRQNumber <128){

		}else if(IRQNumber >= 128 && IRQNumber <160){

		}else if(IRQNumber >= 160 && IRQNumber <192){

		}else if(IRQNumber >= 192 && IRQNumber <224){

		}else if(IRQNumber >= 224 && IRQNumber <256){

		}

	}

}
/**
 *
 * @param IRQNumber
 * @param IRQ_Priority
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQ_Priority){
	/*
	 * IPR is a 32 bit register
	 * For each IRQ number, priority can range from 0 to 255
	 * So, priority requires 8 bits
	 * Each IPR register can define priority for 4 IRQ lines 4*8 = 32 bits
	 * There are 240 External Interrupts, so 60 IPR registers are existing IPR_1_59
	 */

	/*
	 *  Find the ipr register (0 - 59)
	 *  For ex: if IRQNumber = 13
	 *  - then 13/4 = 3 => IPR3 register is to be selected
	 *  - 13%4 = 1 section of IPR3 is to be written with the priority number
	 *  Note: STM implements only the higher 4 bits of the each section in IPR and only allows
	 *  upto 16 programmable priority levels. The lower four bits are non-implemented
	 *  So the bit have to be further shifted by 4 positions
	 */

	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_position = (8*iprx_section) + (8 - NO_OF_PRIBITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + iprx*4) |= (IRQ_Priority << shift_position );

}

/**
 *
 * @param PinNum
 */
void GPIO_IRQHandling(uint8_t PinNum){
	if(EXTI->PR & (1 << PinNum)){
		//clear
		EXTI->PR |= (1<< PinNum);
	}
}


