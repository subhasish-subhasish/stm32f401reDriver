
#include "stm32f401re_gpio.h"

//1. enable clock using calling the macro which is deined in header file
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB)
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
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx==GPIOB)
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
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}
//2. intilize the port
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t position = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	// 1. Enable peripheral clock (assume you have a RCC_EnableGPIOClock() function)
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	 // 2. Configure pin mode (MODER)
	 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode   <=  GPIO_MODE_ANALOG)
	 {
		 pGPIOHandle->pGPIOx->MODER &= ~(0x3U << (2 * position)); // clear
		 pGPIOHandle->pGPIOx->MODER |= ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode & 0x3U) << (2 * position));
	 }else
	 {
		 //Interrupt mode
		 if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		 {
			 //1. configure the FTSR
			 EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 //Clear the corresponding RTSR bit (better for clear corresponding RTSR bit)
			 EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		 {
			 //1 . configure the RTSR
			 EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			 //Clear the corresponding FTSR bit
			 EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		 {
			//1. configure both FTSR and RTSR
		 	EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 	EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		 }
		 //2. configure the GPIO port selection in SYSCFG_EXTICR
		 uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		 uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);
		 //3 . enable the EXIT interrupt delivery using IMR
		 EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	 }
	 // 3. Configure output type (OTYPER)
	 pGPIOHandle->pGPIOx->OTYPER &= ~(0x1U << position);
	 pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << position);

	 // 4. Configure speed (OSPEEDR)
	 pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << (2 * position));
	 pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * position));

	 // 5. Configure pull-up/pull-down (PUPDR)
	 pGPIOHandle->pGPIOx->PUPDR &= ~(0x3U << (2 * position));
	 pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (2 * position));


	 // 6. Configure alternate function (AFRL or AFRH)
	 if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	 {
		 uint32_t afr_index = position / 8;  // 0 = AFRL, 1 = AFRH
	     uint32_t afr_pos   = position % 8;

	     pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xFU << (4 * afr_pos));
	     pGPIOHandle->pGPIOx->AFR[afr_index] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * afr_pos));
	 }
}


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
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
   uint8_t value;

   value = (uint8_t )((pGPIOx->IDR  >> PinNumber) & 0x00000001 ) ;

   return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR  = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0(Interrupt set-enable register 0) register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1(Interrupt set-enable register 1) register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 (Interrupt set-enable register 1) register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0(Interrupt clear-enable register 0) register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1(Interrupt clear-enable register 1) register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2(Interrupt clear-enable register 2) register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the interrypt priority register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);			//Between 8 bit only first 4 bit of msb are uses for priority selection

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);     //Note : This bit is cleared by programming it to ‘1’.
	}
}


