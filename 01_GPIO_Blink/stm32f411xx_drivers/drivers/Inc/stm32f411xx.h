/*
 * stm32f411xx.h
 *
 *  Created on: Jan 14, 2026
 *      Author: dohyeopsong
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

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
#define I2C1_BASEADDR		(APB1PERIPH_BASEDDR + 0x5400)



/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */





#endif /* INC_STM32F411XX_H_ */
