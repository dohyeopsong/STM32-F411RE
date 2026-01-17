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
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0800) // GPIOC
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEDDR + 0x0C00) // GPIOD
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEDDR + 0x1000) // GPIOE
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEDDR + 0x1C00) // GPIOH

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASE			(APB1PERIPH_BASEDDR + 0x5400)
#define I2C2_BASE			(APB1PERIPH_BASEDDR + 0x5800)
#define I2C3_BASE			(APB1PERIPH_BASEDDR + 0x5C00)

#define SPI2_BASE			(APB1PERIPH_BASEDDR + 0x3800)
#define SPI3_BASE			(APB1PERIPH_BASEDDR + 0x3C00)

#define USART2_BASE			(APB1PERIPH_BASEDDR + 0x4400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASE			(APB2PERIPH_BASEDDR + 0x3C00)
#define SYSCFG_BASE			(APB2PERIPH_BASEDDR + 0x3800)
#define SPI4_BASE			(APB2PERIPH_BASEDDR + 0x3400)
#define SPI1_BASE			(APB2PERIPH_BASEDDR + 0x3000)

#define USART1_BASE			(APB2PERIPH_BASEDDR + 0x1000)
#define USART6_BASE			(APB2PERIPH_BASEDDR + 0x1400)


/*********************peripheral register definition structures******************************/

typedef struct
{
	__vo uint32_t MODER;		// GPIO port mode register											offset: 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register									offset: 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register									offset: 0x08
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register								offset: 0x0C
	__vo uint32_t IDR;			// GPIO port input data register									offset: 0x10
	__vo uint32_t ODR;			// GPIO port output data register									offset: 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register									offset: 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register							offset: 0x1C
	__vo uint32_t AFR[2];		// GPIO alternate function low: ARF[0]/high: ARF[1] register		offset: 0x20 - 0x24
}GPIO_RegDef_t;









#endif /* INC_STM32F411XX_H_ */
