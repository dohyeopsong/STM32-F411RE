/*
 * stm32f411xx.h
 *
 *  Created on: Jan 14, 2026
 *      Author: dohyeopsong
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

#define __vo  volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0			((__vo uint32_5*)0xE00E100)
#define NVIC_ISER1			((__vo uint32_5*)0xE00E104)
#define NVIC_ISER2			((__vo uint32_5*)0xE00E108)
#define NVIC_ISER3			((__vo uint32_5*)0xE00E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE00E400)

/*
 * ARM Cortex Mx Processor number of priority bits impleamented in Priority
 */
#define NO_PR_BITS_IMPLEMENTED		4

// base addresses of Flash and SRAM memories
#define FLASH_BASEADDR      0x08000000U   // 플래시 메모리 시작 주소
#define SRAM1_BASEADDR      0x20000000U   // SRAM1 시작 주소, 12KB
#define ROM_BASEADDR        0x1FFF0000U   // 시스템 메모리(ROM) 시작 주소
#define SRAM_BASEADDR       SRAM1_BASEADDR // 메인 SRAM은 SRAM1을 의미함


// AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASEDDR			0x40000000U
#define APB1PERIPH_BASEDDR		PERIPH_BASEDDR
#define APB2PERIPH_BASEDDR		0x40010000U
#define AHB1PERIPH_BASEDDR 		0x40020000U
#define AHB2PERIPH_BASEDDR		0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0000) // GPIOA
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0400) // GPIOB
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0800) // GPIOC
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0C00) // GPIOD
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEDDR + 0x1000) // GPIOE
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEDDR + 0x1C00) // GPIOH
#define RCC_BASEADDR		(AHB1PERIPH_BASEDDR + 0x3800) // RCC
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASEDDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEDDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEDDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEDDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEDDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEDDR + 0x4400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR			(APB2PERIPH_BASEDDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEDDR + 0x3800)
#define SPI4_BASEADDR			(APB2PERIPH_BASEDDR + 0x3400)
#define SPI1_BASEADDR			(APB2PERIPH_BASEDDR + 0x3000)
#define SPI5_BASEADDR			(APB2PERIPH_BASEDDR + 0x5000)

#define USART1_BASEADDR			(APB2PERIPH_BASEDDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEDDR + 0x1400)


/*********************peripheral register definition structures******************************/

typedef struct
{
	__vo uint32_t MODER;		// GPIO port mode register											Address offset: 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register									Address offset: 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register									Address offset: 0x08
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register								Address offset: 0x0C
	__vo uint32_t IDR;			// GPIO port input data register									Address offset: 0x10
	__vo uint32_t ODR;			// GPIO port output data register									Address offset: 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register									Address offset: 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register							Address offset: 0x1C
	__vo uint32_t AFR[2];		// GPIO alternate function low: ARF[0]/high: ARF[1] register		Address offset: 0x20 - 0x24
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			   // RCC clock control register												Address offset: 0x00
	__vo uint32_t PLLCFGR;		   // RCC PLL configuration register											Address offset: 0x04
	__vo uint32_t CFGR;			   // RCC clock configuration register											Address offset: 0x08
	__vo uint32_t CIR;			   // RCC clock interrupt register											    Address offset: 0x0C
	__vo uint32_t AHB1RSTR;		   // RCC AHB1 peripheral reset register										Address offset: 0x10
	__vo uint32_t AHB2RSTR;		   // RCC AHB2 peripheral reset register										Address offset: 0x14
	uint32_t	  RESERVED0[2];	   // Reserved, 0x18-0x1C
	__vo uint32_t APB1RSTR;		   // RCC APB1 peripheral reset register for									Address offset: 0x20
	__vo uint32_t APB2RSTR;		   // RCC APB2 peripheral reset register										Address offset: 0x24
	uint32_t	  RESERVED1[2];	   // Reserved, 0x28-0x2C
	__vo uint32_t AHB1ENR;		   // RCC AHB1 peripheral clock enable register								    Address offset: 0x30
	__vo uint32_t AHB2ENR;		   // RCC AHB2 peripheral clock enable register								    Address offset: 0x34
	uint32_t	  RESERVED2[2];	   // Reserved, 0x38-0x3C
	__vo uint32_t APB1ENR;		   // RCC APB1 peripheral clock enable register								    Address offset: 0x40
	__vo uint32_t APB2ENR;		   // RCC APB2 peripheral clock enable register								    Address offset: 0x44
	uint32_t	  RESERVED3[2];	   // Reserved, 0x48-0x4C
	__vo uint32_t AHB1LPENR;	   // TRCC AHB1 peripheral clock enable in low power mode register				Address offset: 0x50
	__vo uint32_t AHB2LPENR;	   // RCC AHB2 peripheral clock enable in low power mode register				Address offset: 0x54
	uint32_t	  RESERVED4[2];	   // Reserved, 0x58-0x5C
	__vo uint32_t APB1LPENR;	   // RCC APB1 peripheral clock enable in low power mode register				Address offset: 0x60
	__vo uint32_t APB2LPENR;	   // RCC APB2 peripheral clock enabled in low power mode register				Address offset: 0x64
	uint32_t	  RESERVED5[2];	   // Reserved, 0x68-0x6C
	__vo uint32_t BDCR;			   // RCC Backup domain control register										Address offset: 0x70
	__vo uint32_t CSR;			   // RCC clock control & status register										Address offset: 0x74
	uint32_t	  RESERVED6[2];	   // Reserved, 0x78-0x7C
	__vo uint32_t SSCGR;		   // RCC spread spectrum clock generation register							    Address offset: 0x80
	__vo uint32_t PLLI2SCFGR;	   // RCC PLLI2S configuration register										    Address offset: 0x84
	uint32_t	  RESERVED7;	   // Reserved, 0x88
	__vo uint32_t DCKCFGR;		   // RCC Dedicated Clocks Configuration Register								Address offset: 0x8C
}RCC_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;		// EXTI Interrupt mask register 									Address offset: 0x00
	__vo uint32_t EMR;		// EXTI Event mask register											Address offset: 0x04
	__vo uint32_t RTSR;		// EXTI Rising trigger selection register							Address offset: 0x08
	__vo uint32_t FTSR;		// EXTI Falling trigger selection register							Address offset: 0x0C
	__vo uint32_t SWIER;	// EXTI Software interrupt event register							Address offset: 0x10
	__vo uint32_t PR;		// EXTI Pending register											Address offset: 0x14
}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;		// SYSCFG memory remap register 									Address offset: 0x00
	__vo uint32_t PMC;			// SYSCFG peripheral mode configuration register					Address offset: 0x04
	__vo uint32_t EXTICR[4];	// SYSCFG external interrupt configuration register 1~4				Address offset: 0x08-0x14
	uint32_t RESERVED[2];		// Reserved, 0x18-0x1C
	__vo uint32_t CMPCR;		// Compensation cell control register								Address offset: 0x20
}SYSCFG_RegDef_t;
/***************peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )**********************/
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t)SYSCFG_BASEADDR)
/*********************Clock Enable Macros******************************/
// Clock Enable Macros for GPIOx peripherals
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

// Clock Enable Macros for I2Cx peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


// Clock Enable Macros for SPIx peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN() (RCC->APB2ENR |= (1 << 20))

// Clock Enable Macros for USARTx peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

// Clock Enable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

/*********************Clock Disable Macros******************************/
// Clock Disable Macros for GPIOx peripherals
#define GPIOA_PCLK_DI()  	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))

// Clock Disable Macros for I2Cx peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

// Clock Disable Macros for SPIx peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI() (RCC->APB2ENR &= ~(1 << 20))

// Clock Disable Macros for USARTx peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))

// Clock Disable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


#define GPIO_BASEADDR_TO_CODE(x)		  ( (x == GPIOA) ? 0:\
										    (x == GPIOB) ? 1:\
										    (x == GPIOC) ? 2:\
											(x == GPIOD) ? 3:\
											(x == GPIOE) ? 4:\
											(x == GPIOH) ? 7:0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F411xx MCU
 */

#define IRQ_NO_EXTI16		1
#define IRQ_NO_EXTI21		2
#define IRQ_NO_EXTI22		3
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_EXTI17		41
#define IRQ_NO_EXTI18		42

//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         	RESET
#define FLAG_SET 			SET




#include "stm32f411xx_gpio_driver.h"

#endif /* INC_STM32F411XX_H_ */
