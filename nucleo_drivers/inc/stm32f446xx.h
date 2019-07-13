/*
 * stm32f446xx.h
 *
 *  Created on: Jun 25, 2019
 *      Author: Nandan
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

/* Processor specific details - Cortex M4 */

#define NO_OF_PRIBITS_IMPLEMENTED		4

/*
 * ARM Cortex M4 processor NVIC ISERx(Interrupt Set Enable) register addresses
 */

#define NVIC_ISER0				((volatile uint32_t*) 0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*) 0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*) 0xE000E108)


/*
 * ARM Cortex M4 processor NVIC ICERx(Interrupt Clear Enable) register addresses
 */

#define NVIC_ICER0				((volatile uint32_t*) 0xE000E180)
#define NVIC_ICER1				((volatile uint32_t*) 0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*) 0xE000E188)

/*
 * ARM Cortex M4 processor NVIC IPRx register (Interrupt Priority Reg) base address
 */

#define NVIC_IPR_BASEADDR				((volatile uint32_t*) 0xE000E400)


/* Board specific details - STM32F446 */

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
#define RCC_BASEADDR				(AHB1_BASEADDR + 0X3800)

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
	volatile uint32_t MODER; 		/* GPIO port mode register,	Address offset 0x00 */
	volatile uint32_t OTYPER;		/* GPIO port output type register,	Address offset 0x04 */
	volatile uint32_t OSPEEDER;		/* GPIO port output speed register,	Address offset 0x08 */
	volatile uint32_t PUPDR;			/* GPIO port pull-up/pull-down register,	Address offset 0x0C */
	volatile uint32_t IDR;			/* GPIO port input data register,	Address offset 0x10 */
	volatile uint32_t ODR;			/* GPIO port output data register,	Address offset 0x14 */
	volatile uint32_t BSRR;			/* GPIO port bit set/reset register,	Address offset 0x18 */
	volatile uint32_t LCKR;			/* GPIO port configuration lock register,	Address offset 0x1C */
	volatile uint32_t AFR[2];		/* AF[0]: GPIO alternate function low register, AF[1]: GPIO alternate function high registerAddress offset 0x20 */
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t PLLCGFR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

}RCC_RegDef_t;

typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	uint32_t RESERVED2[2];
	volatile uint32_t CFGR;

}SYSCFG_RegDef_t;


typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;

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

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |=  (1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |=  (1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |=  (1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |=  (1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |=  (1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |=  (1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |=  (1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |=  (1<<7))

/*
 * Register reset macros for GPIOx peripherals
 */

#define GPIOA_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_RST_REG()		do{ (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Clock enable macros for SPI
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1<<13))

/*
 * Clock enable macros for I2C
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock enable macros for USART
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<6))

/*
 * Clock enable macros for SYSCFG
 */

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &=  ~(1<<7))

/*
 * Clock disable macros for SPI
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1<<13))
/*
 * Clock disable macros for I2C
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))
/*
 * Clock disable macros for USART
 */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<6))


/*
 * Clock disable macros for SYSCFG
 */

#define SYSCFG_DI()			(RCC->APB2ENR &= ~(1<<14))

/*
 * Returns the port name for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)? 0:\
										 (x == GPIOB)? 1:\
										 (x == GPIOC)? 2:\
										 (x == GPIOD)? 3:\
										 (x == GPIOE)? 4:\
										 (x == GPIOF)? 5:\
										 (x == GPIOG)? 6:\
										 (x == GPIOH)? 7:0)

/*
 * IRQ numbers on the processor for external interrupt lines
 */

#define IRQ_EXTI0		6
#define IRQ_EXTI1		7
#define IRQ_EXTI2		8
#define IRQ_EXTI3		9
#define IRQ_EXTI4		10
#define IRQ_EXTI9_5		23
#define IRQ_EXTI15_10	40

/*
 * Some generic macros
 */

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE


#include "stm32f446xx_gpio_driver.h"


#endif /* INC_STM32F446XX_H_ */
