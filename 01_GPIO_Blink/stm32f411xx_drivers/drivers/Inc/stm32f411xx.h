/*
 * stm32f411xx.h
 *
 *  Created on: Jan 14, 2026
 *      Author: dohyeopsong
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_


#define FLASH_BASEADDR      0x08000000U   // 플래시 메모리 시작 주소
#define SRAM1_BASEADDR      0x20000000U   // SRAM1 시작 주소, 12KB
#define ROM_BASEADDR        0x1FFF0000U   // 시스템 메모리(ROM) 시작 주소
#define SRAM_BASEADDR       SRAM1_BASEADDR // 메인 SRAM은 SRAM1을 의미함


#endif /* INC_STM32F411XX_H_ */
