
#ifndef DRIVER_INC_STM32F401RE_RCC_H_
#define DRIVER_INC_STM32F401RE_RCC_H_


#include "stm32f401re.h"

// Enumeration for Clock Sources
typedef enum {
    RCC_CLOCK_HSI = 0,   // High Speed Internal (16 MHz)
    RCC_CLOCK_HSE = 1,   // High Speed External (crystal/oscillator)
    RCC_CLOCK_PLL = 2    // Phase Locked Loop
} RCC_ClockSource_t;
// Enumeration for AHB Prescaler
typedef enum {
    RCC_AHB_DIV1   = 0x0,
    RCC_AHB_DIV2   = 0x8,
    RCC_AHB_DIV4   = 0x9,
    RCC_AHB_DIV8   = 0xA,
    RCC_AHB_DIV16  = 0xB,
    RCC_AHB_DIV64  = 0xC,
    RCC_AHB_DIV128 = 0xD,
    RCC_AHB_DIV256 = 0xE,
    RCC_AHB_DIV512 = 0xF
} RCC_AHB_Prescaler_t;
// Enumeration for APB Prescaler
typedef enum {
    RCC_APB_DIV1  = 0x0,  // 000: HCLK not divided
    RCC_APB_DIV2  = 0x4,  // 100: HCLK divided by 2
    RCC_APB_DIV4  = 0x5,  // 101: HCLK divided by 4
    RCC_APB_DIV8  = 0x6,  // 110: HCLK divided by 8
    RCC_APB_DIV16 = 0x7   // 111: HCLK divided by 16
} RCC_APB_Prescaler_t;




typedef enum {
    RCC_PLLSRC_HSI = 0,   // HSI as PLL input
    RCC_PLLSRC_HSE        // HSE as PLL input
} RCC_PLLSource_t;
// PLL Config
typedef struct {
	RCC_PLLSource_t PLL_Source;
    uint8_t PLLM;   // Division factor for PLL input clock
    uint16_t PLLN;  // Multiplication factor for VCO
    uint8_t PLLP;   // Division factor for main system clock
    uint8_t PLLQ;   // Division factor for USB/SDIO/RNG clocks
} RCC_PLL_Config_t;

// Main RCC Config structure
typedef struct {
    RCC_ClockSource_t ClockSource;     // HSI, HSE, or PLL
    RCC_PLL_Config_t PLL_Config;       // PLL settings (if PLL is used)
    RCC_AHB_Prescaler_t AHB_Prescaler; // Prescaler for AHB (HCLK)
    RCC_APB_Prescaler_t APB1_Prescaler;// Prescaler for APB1 (PCLK1)
    RCC_APB_Prescaler_t APB2_Prescaler;// Prescaler for APB2 (PCLK2)
} RCC_Config_t;



/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
void ClockConfig(RCC_Config_t * RCC_Init);



//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);














#endif /* DRIVER_INC_STM32F401RE_RCC_H_ */
