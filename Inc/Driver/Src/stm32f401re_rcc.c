#include "stm32f401re_rcc.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = { 2, 4 , 8, 16};

static void RCC_SetAHBPrescaler(RCC_AHB_Prescaler_t prescaler)
{
    // Clear HPRE[3:0] bits (bits 7:4 of RCC->CFGR)
    RCC->CFGR &= ~(0xF << RCC_CFGR_HPRE);
    // Set new prescaler value
    RCC->CFGR |= (prescaler << RCC_CFGR_HPRE);
}

static void RCC_SetAPB1Prescaler(RCC_APB_Prescaler_t prescaler)
{
    // Clear PPRE1[2:0] bits (bits 10:8)
    RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE1);
    // Set new prescaler
    RCC->CFGR |= (prescaler << RCC_CFGR_PPRE1);
}
static void RCC_SetAPB2Prescaler(RCC_APB_Prescaler_t prescaler)
{
    // Clear PPRE2[2:0] bits (bits 13:11)
    RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE2);
    // Set new prescaler
    RCC->CFGR |= (prescaler << RCC_CFGR_PPRE2);
}

static void RCC_SetSystemClock(RCC_ClockSource_t clockSource)
{
    // Clear SW[1:0] bits (System clock switch)
    RCC->CFGR &= ~(0x3 << RCC_CFGR_SW);

    // Set new clock source
    RCC->CFGR |= (clockSource << RCC_CFGR_SW);

    // Wait until selected clock source is used as system clock
    while (((RCC->CFGR >> RCC_CFGR_SWS) & 0x3) != clockSource);
}

static void RCC_ConfigPLL(const RCC_PLL_Config_t *pllCfg)
{
    // 1. Disable PLL before configuring
    RCC->CR &= ~(1 << RCC_CR_PLLON);  // PLLON = 0
    while (RCC->CR & (1 << RCC_CR_PLLRDY)); // Wait PLLRDY = 0

    // 2. Configure PLL source, M, N, P, Q
    RCC->PLLCFGR = 0;  // Reset first
    RCC->PLLCFGR |= (pllCfg->PLL_Source << RCC_PLLCFGR_PLLSRC);        // PLLSRC bit
    RCC->PLLCFGR |= (pllCfg->PLLM & 0x3F) << RCC_PLLCFGR_PLLM;            // PLLM (bits 5:0)
    RCC->PLLCFGR |= (pllCfg->PLLN & 0x1FF) << RCC_PLLCFGR_PLLN;      // PLLN (bits 14:6)
    RCC->PLLCFGR |= ((pllCfg->PLLP >> 1) - 1) << RCC_PLLCFGR_PLLP;  // PLLP (bits 17:16)
    RCC->PLLCFGR |= (pllCfg->PLLQ & 0x0F) << RCC_PLLCFGR_PLLQ;      // PLLQ (bits 27:24)

    // 3. Enable PLL
    RCC->CR |= (1 << RCC_CR_PLLON);  // PLLON = 1
    while (!(RCC->CR & (1 << RCC_CR_PLLRDY))); // Wait until PLLRDY = 1
}


void ClockConfig(RCC_Config_t *RCC_Init)
{

	RCC_ConfigPLL(&RCC_Init->PLL_Config);
	RCC_SetSystemClock(RCC_Init->ClockSource);
	RCC_SetAHBPrescaler(RCC_Init->AHB_Prescaler);
	RCC_SetAPB1Prescaler(RCC_Init->APB1_Prescaler);
	RCC_SetAPB2Prescaler(RCC_Init->APB2_Prescaler);
}

uint32_t RCC_GetSystemClkVal(void)
{
	uint32_t SystemClk;
	uint8_t clksrc;
	clksrc = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);
	if(clksrc == 0 )
	{
		SystemClk = HSI_VALUE;
	}else if(clksrc == 1)
	{
		SystemClk = HSE_VALUE;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	return SystemClk;
}

uint32_t RCC_GetAHBClkVal(void)
{
	uint32_t SystemClk;
	uint8_t temp,ahbp;
	SystemClk = RCC_GetSystemClkVal();
	temp = ((RCC->CFGR >> RCC_CFGR_HPRE ) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	return SystemClk/ahbp;
}

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t AHBClk;
	AHBClk = RCC_GetAHBClkVal();
	uint8_t temp,apb1p;
	temp = ((RCC->CFGR >> RCC_CFGR_PPRE1 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB_PreScaler[temp-4];
	}
	return  AHBClk /apb1p;
}

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t AHBClk;
	AHBClk = RCC_GetAHBClkVal();
	uint8_t temp,apb2p;

	temp = ((RCC->CFGR >>RCC_CFGR_PPRE2) & 0xf);
	if(temp < 4)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB_PreScaler[temp-4];
	}

	return AHBClk /apb2p;
}
uint32_t  RCC_GetPLLOutputClock(void)
{
	uint32_t pllcfgr = RCC->PLLCFGR;

	uint32_t pllsrc = (pllcfgr >> RCC_PLLCFGR_PLLSRC) & 0x1;
	uint32_t f_in = (pllsrc == 0) ? HSI_VALUE : HSE_VALUE;

	uint32_t pllm = (pllcfgr >> RCC_PLLCFGR_PLLM) & 0x3F;
	uint32_t plln = (pllcfgr >> RCC_PLLCFGR_PLLN) & 0x1FF;
	uint32_t pllp = (((pllcfgr >> RCC_PLLCFGR_PLLP) & 0x3) + 1) * 2;

	uint32_t vco_freq = (f_in / pllm) * plln;

	return vco_freq / pllp;

}

