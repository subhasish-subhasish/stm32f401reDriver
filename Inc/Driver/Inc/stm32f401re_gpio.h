/*
 * stm32f401re_gpio.h
 *
 *  Created on: May 25, 2025
 *      Author: subha
 */

#ifndef DRIVER_INC_STM32F401RE_GPIO_H_
#define DRIVER_INC_STM32F401RE_GPIO_H_

#include <stm32f401re.h>


typedef enum
{
    GPIO_MODE_IN     = 0x0,  // Input mode
    GPIO_MODE_OUT    = 0x1,  // Output mode
    GPIO_MODE_ALTFN  = 0x2,  // Alternate Function mode
    GPIO_MODE_ANALOG = 0x3,  // Analog mode
	// Interrupt modes (software-defined, not in MODER register)
	GPIO_MODE_IT_FT   = 0x4,  // Input Falling edge trigger
	GPIO_MODE_IT_RT   = 0x5,  // Input Rising edge trigger
	GPIO_MODE_IT_RFT  = 0x6   // Input Rising + Falling edge trigger
} GPIO_Mode_t;
typedef enum
{
    GPIO_PIN_0  = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
} GPIO_PinNumber_t;
typedef enum
{
    GPIO_NO_PUPD = 0x0,
    GPIO_PIN_PU  = 0x1,
    GPIO_PIN_PD  = 0x2
} GPIO_PuPd_t;
typedef enum
{
    GPIO_SPEED_LOW       = 0x0,
    GPIO_SPEED_MEDIUM    = 0x1,
    GPIO_SPEED_FAST      = 0x2,
    GPIO_SPEED_HIGH      = 0x3
} GPIO_Speed_t;
typedef enum
{
    GPIO_OP_TYPE_PP = 0x0,  // Push-Pull
    GPIO_OP_TYPE_OD = 0x1   // Open-Drain
} GPIO_OutputType_t;
typedef struct
{
	GPIO_PinNumber_t    GPIO_PinNumber;
	GPIO_Mode_t         GPIO_PinMode;
	GPIO_Speed_t        GPIO_PinSpeed;
	GPIO_PuPd_t         GPIO_PinPuPd;
	GPIO_OutputType_t   GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//This is a Handle structure for a GPIO pin
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;







/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

//Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */

