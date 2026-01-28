/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jan 18, 2026
 *      Author: dohyeopsong
 */

#include "stm32f411xx_gpio_driver.h"



/******************************************************************************************************************************************
 * Peripheral Clock setup
 * @fn          - GPIO_PeriClockControl
 * @brief       - This function enables or disables the peripheral clock for the given GPIO port. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral (e.g., GPIOA, GPIOB, etc.). [cite: 10, 11]
 * @param[in]   - EnorDi: ENABLE or DISABLE macros. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Direct manipulation of RCC (Reset and Clock Control) registers is performed. [cite: 10, 11]
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}


/******************************************************************************************************************************************
 * Init and De-init
 * @fn          - GPIO_Init
 * @brief       - This function initializes the GPIO pin mode, speed, output type, pull-up/pull-down, and alternate functions. [cite: 10, 11]
 * @param[in]   - pGPIOHandle: Pointer to the handle structure containing port address and pin configurations. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - This function also handles EXTI (External Interrupt) configuration including edge triggers and IMR. [cite: 10, 11]
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; //temp. register

	// 1. configure the mode of gpio pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
		pGPIOHandle->pGPIOx->MODER |= temp; // setting
	}else
	{
		// this part will code later. (interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. configure the FTSR(falling trigger selection register)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 // Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. configure the RTST
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. configure both FTSR and STSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. confugyre the GPIO part selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG.EXTICR[temp1] |= portcode << (temp2 * 4);

		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// 5. configure the alt functionaliy
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt funtion registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/******************************************************************************************************************************************
 * @fn          - GPIO_DeInit
 * @brief       - This function resets all registers of a specific GPIO port to their default values. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral to be reset. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Utilizes the RCC_AHB1RSTR (AHB1 Peripheral Reset Register) to reset the entire port. [cite: 10, 11]
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/******************************************************************************************************************************************
 * Data read and write
 * @fn          - GPIO_ReadFromInputPin
 * @brief       - Reads the input data (0 or 1) from a specific pin of a GPIO port. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral. [cite: 10, 11]
 * @param[in]   - PinNumber: The pin number to read from (0 to 15). [cite: 10, 11]
 * @return      - 0 or 1 (uint8_t). [cite: 10, 11]
 * @Note        - Accesses the IDR (Input Data Register) to check the bit status. [cite: 10, 11]
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/******************************************************************************************************************************************
 * @fn          - GPIO_ReadFromInputPort
 * @brief       - Reads the entire 16-bit input data from a specific GPIO port. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral. [cite: 10, 11]
 * @return      - The 16-bit status value of the entire port (uint16_t). [cite: 10, 11]
 * @Note        - Returns the raw value of the IDR register. [cite: 10, 11]
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

		value = (uint16_t) pGPIOx->IDR;

		return value;
}

/******************************************************************************************************************************************
 * @fn          - GPIO_WriteToOutputPin
 * @brief       - Writes a value (SET or RESET) to a specific GPIO pin. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral. [cite: 10, 11]
 * @param[in]   - PinNumber: The pin number to write to (0 to 15). [cite: 10, 11]
 * @param[in]   - Value: GPIO_PIN_SET or GPIO_PIN_RESET. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Modifies the ODR (Output Data Register) to set the voltage level. [cite: 10, 11]
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		// write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}
/******************************************************************************************************************************************
 * @fn          - GPIO_WriteToOutputPort
 * @brief       - Writes a 16-bit value to the entire GPIO port. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral. [cite: 10, 11]
 * @param[in]   - Value: The 16-bit value to be written to the port. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Updates the entire ODR register with the provided value. [cite: 10, 11]
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/******************************************************************************************************************************************
 * @fn          - GPIO_ToggleOutputPin
 * @brief       - Toggles the output state (0 to 1 or 1 to 0) of a specific GPIO pin. [cite: 10, 11]
 * @param[in]   - pGPIOx: Base address of the GPIO peripheral. [cite: 10, 11]
 * @param[in]   - PinNumber: The pin number to toggle (0 to 15). [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Uses the XOR (^) operator on the ODR register bit. [cite: 10, 11]
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}

/******************************************************************************************************************************************
 * IRQ Configuration and ISR handling
 * @fn          - GPIO_IRQInterruptConfig
 * @brief       - Configures the NVIC registers to enable or disable a specific IRQ number. [cite: 10, 11]
 * @param[in]   - IRQNumber: The position/IRQ number of the interrupt to be configured. [cite: 10, 11]
 * @param[in]   - EnorDi: ENABLE or DISABLE. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Refers to Cortex-M4 ISER/ICER registers as described in the Generic User Guide. [cite: 10, 11]
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64) // 32 to 63
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			// program ICER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

/******************************************************************************************************************************************
 * @fn          - GPIO_IRQPriorityConfig
 * @brief       - Sets the priority for a specific IRQ number in the NVIC. [cite: 10, 11]
 * @param[in]   - IRQNumber: The position/IRQ number to set priority for. [cite: 10, 11]
 * @param[in]   - IRQPriority: Priority value (0 to 15). [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Note: STM32 utilizes only the upper 4 bits of the 8-bit priority field. [cite: 10, 11]
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/******************************************************************************************************************************************
 * @fn          - GPIO_IRQHandling
 * @brief       - Handles the interrupt by clearing the pending bit in the EXTI register. [cite: 10, 11]
 * @param[in]   - PinNumber: The pin number associated with the interrupt. [cite: 10, 11]
 * @return      - None. [cite: 10, 11]
 * @Note        - Clearing the pending bit is mandatory to prevent the interrupt from re-triggering infinitely. [cite: 10, 11]
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
