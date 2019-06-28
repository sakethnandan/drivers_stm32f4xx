/*
 * stm32f446xx.h
 *
 *  Created on: Jun 25, 2019
 *      Author: Nandan
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

#define __vo volatile

/*
 * Base addresses of flash and SRAM
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM 						SRAM1_BASEADDR

/*
 * Base addresses of busses
 */
#define PERI_BASEADDR				0x40000000U

#define APB1_BASEADDR				PERI_BASEADDR
#define APB2_BASEADDR				0x40010000U
#define AHB1_BASEADDR				0x40020000U
#define AHB2_BASEADDR				0x50000000U
#define AHB3_BASEADDR				0xA0001000U


/*
 * Base addresses of peripherals connected to AHB1
 */

#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)

/*
 * Base addresses of peripherals connected to APB1
 */

#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1_BASEADDR + 0x4800)

#define UART4_BASEADDR				(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1_BASEADDR + 0x5000)

#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)

/*
 * Base addresses of peripherals connected to APB2
 */

#define USART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2_BASEADDR + 0x1400)

#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)

/*
 * Peripheral register structures
 */

typedef struct{
	__vo uint32_t MODER; 		/* GPIO port mode register,	Address offset 0x00 */
	__vo uint32_t OTYPER;		/* GPIO port output type register,	Address offset 0x04 */
	__vo uint32_t OSPEEDER;		/* GPIO port output speed register,	Address offset 0x08 */
	__vo uint32_t PUPDR;			/* GPIO port pull-up/pull-down register,	Address offset 0x0C */
	__vo uint32_t IDR;			/* GPIO port input data register,	Address offset 0x10 */
	__vo uint32_t ODR;			/* GPIO port output data register,	Address offset 0x14 */
	__vo uint32_t BSRR;			/* GPIO port bit set/reset register,	Address offset 0x18 */
	__vo uint32_t LCKR;			/* GPIO port configuration lock register,	Address offset 0x1C */
	__vo uint32_t AFR[2];		/* AF[0]: GPIO alternate function low register, AF[1]: GPIO alternate function high registerAddress offset 0x20 */
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t RCC_CR; 		/* GPIO port mode register,	Address offset 0x00 */
	__vo uint32_t RCC_PLLCGFR;		/* GPIO port output type register,	Address offset 0x04 */
	__vo uint32_t RCC_CFGR;		/* GPIO port output speed register,	Address offset 0x08 */
	__vo uint32_t RCC_CIR;			/* GPIO port pull-up/pull-down register,	Address offset 0x0C */
	__vo uint32_t RCC_AHB1RSTR;			/* GPIO port input data register,	Address offset 0x10 */
	__vo uint32_t RCC_AHB2RSTR;			/* GPIO port output data register,	Address offset 0x14 */
	__vo uint32_t RCC_AHB3RSTR;			/* GPIO port bit set/reset register,	Address offset 0x18 */
	__vo uint32_t RCC_APB1RSTR;			/* GPIO port configuration lock register,	Address offset 0x1C */
	__vo uint32_t AFR[2];		/* AF[0]: GPIO alternate function low register, AF[1]: GPIO alternate function high registerAddress offset 0x20 */
}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)



#endif /* INC_STM32F446XX_H_ */
