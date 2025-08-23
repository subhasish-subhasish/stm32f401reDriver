

#include <stdint.h>
#include <stm32f401re.h>



void clkConfig(void);

void GpioConfig(void);


int main(void)
{
	clkConfig();

	GpioConfig();

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_0);
	}



	return 0;

}
void clkConfig(void)
{
	RCC_Config_t RCC_Config;

	RCC_Config.PLL_Config.PLL_Source = RCC_PLLSRC_HSI;
	RCC_Config.PLL_Config.PLLM = 16;
	RCC_Config.PLL_Config.PLLN =168;
	RCC_Config.PLL_Config.PLLP =2;
	RCC_Config.ClockSource = RCC_CLOCK_PLL;
	RCC_Config.AHB_Prescaler = RCC_AHB_DIV1;
	RCC_Config.APB1_Prescaler = RCC_APB_DIV2;
	RCC_Config.APB2_Prescaler = RCC_APB_DIV1;

	ClockConfig(&RCC_Config);
}

void GpioConfig(void)
{
	GPIO_Handle_t GpioLed;

		GpioLed.pGPIOx = GPIOA;
		GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
		GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		GpioLed.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
		GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

		GPIO_Init(&GpioLed);
}

