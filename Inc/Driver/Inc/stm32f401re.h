
#ifndef STM32F401RE_H
#define STM32F401RE_H

#ifdef __cplusplus
 extern "C" {
#endif




#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
//ARM Cortex Mx Processor NVIC ISERx(Interrupt set-enable register x) register Addresses
#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )

//ARM Cortex Mx Processor NVIC ICERx(Interrupt clear-enable register x) register Addresses
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

//ARM Cortex Mx Processor Priority Register Address Calculation
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

//ARM Cortex Mx Processor number of priority bits implemented in Priority Register
#define NO_PR_BITS_IMPLEMENTED  4

//base addresses of Flash and SRAM memories
#define FLASH_BASEADDR						0x08000000U   		//Defines the base address of the internal Flash memory. 512KB
#define SRAM1_BASEADDR						0x20000000U  		//Defines the base address of the first Static Random Access Memory (SRAM1) 96KB.
#define ROM_BASEADDR						0x1FFF0000U			//Defines the base address of the System Memory, often referred to as ROM. 1KB used for booting
#define SRAM 								SRAM1_BASEADDR		//Creates an alias SRAM for the base address of SRAM1.

//AHBx and APBx Bus Peripheral base addresses
#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

//Base addresses of peripherals which are hanging on AHB1 bus
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)

//Base addresses of peripherals which are hanging on APB1 bus
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)


//Base addresses of peripherals which are hanging on APB2 bus
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)


/*************************************************************************************************************
**********************************peripheral register definition structures **********************************
**************************************************************************************************************/



/*********************************************************GPIO************************************************/
typedef struct
{
	__vo uint32_t MODER;		//GPIO port mode register     		:                              									Address offset: 0x00
	__vo uint32_t OTYPER;       //GPIO port output type reg   		:  0: Output push-pull (reset state), 1: Output open-drain     	Address offset: 0x04
	__vo uint32_t OSPEEDR;	    //GPIO port output speed reg	 	:
	__vo uint32_t PUPDR;	    //GPIO port pull-up/pull-down reg 	:
	__vo uint32_t IDR;			//GPIO port input data register		:
	__vo uint32_t ODR;			//GPIO port output data register	:																Address offset: 0x14
	__vo uint32_t BSRR;			//GPIO port bit set/reset register 	:
	__vo uint32_t LCKR;			//GPIO port configuration lock reg	:
	__vo uint32_t AFR[2];		//AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    	Address offset: 0x20-0x24
}GPIO_RegDef_t;

/*********************************************************RCC************************************************/
typedef struct
{
  __vo uint32_t CR;            //clock control register     							Address offset: 0x00
  __vo uint32_t PLLCFGR;       //PLL configuration register     						Address offset: 0x04
  __vo uint32_t CFGR;          //clock configuration register     						Address offset: 0x08
  __vo uint32_t CIR;           //clock interrupt register    							Address offset: 0x0C
  __vo uint32_t AHB1RSTR;      //AHB1 peripheral reset register     					Address offset: 0x10
  __vo uint32_t AHB2RSTR;      //AHB2 peripheral reset register							Address offset: 0x14
  __vo uint32_t AHB3RSTR;      //AHB2 peripheral reset register  						Address offset: 0x18
  uint32_t      RESERVED0;     //Reserved, 0x1C
  __vo uint32_t APB1RSTR;      //APB1 peripheral reset register     					Address offset: 0x20
  __vo uint32_t APB2RSTR;      //APB2 peripheral reset register     					Address offset: 0x24
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       //AHB1 peripheral clock enable register     				Address offset: 0x30
  __vo uint32_t AHB2ENR;       //AHB2 peripheral clock enable register     				Address offset: 0x34
  __vo uint32_t AHB3ENR;       //AHB3 peripheral clock enable register     				Address offset: 0x38
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       //APB1 peripheral clock enable register     				Address offset: 0x40
  __vo uint32_t APB2ENR;       //APB2 peripheral clock enable register     				Address offset: 0x44
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     //AHB1 peripheral clock enable in low power mode reg.	Address offset: 0x50
  __vo uint32_t AHB2LPENR;     //AHB2 peripheral clock enable in low power mode reg.	Address offset: 0x54
  __vo uint32_t AHB3LPENR;     //AHB3 peripheral clock enable in low power mode reg.	Address offset: 0x58
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     //APB1 peripheral clock enable in low power mode reg.	Address offset: 0x60
  __vo uint32_t APB2LPENR;     //APB2 peripheral clock enable in low power mode reg.	Address offset: 0x64
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          //Backup domain control register   						Address offset: 0x70
  __vo uint32_t CSR;           //clock control & status register     					Address offset: 0x74
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         //spread spectrum clock generation register    			Address offset: 0x80
  __vo uint32_t PLLI2SCFGR;    //PLLI2S configuration register							Address offset: 0x84
  __vo uint32_t PLLSAICFGR;    //!< TODO,     											Address offset: 0x88
  __vo uint32_t DCKCFGR;       //Dedicated Clocks Configuration Register     			Address offset: 0x8C
  __vo uint32_t CKGATENR;      //*!< TODO,     											Address offset: 0x90
  __vo uint32_t DCKCFGR2;      //*!< TODO,     											Address offset: 0x94
} RCC_RegDef_t;

/*********************************************************EXIT************************************************/
typedef struct
{
	__vo uint32_t IMR;    //Interrupt Mask Register 			: Masks or unmasks the interrupt request from each EXTI line,         	 	0 – Masked (Interrupt disabled),	1 – Unmasked (Interrupt enabled)  					Address offset: 0x00
	__vo uint32_t EMR;    //Event Mask Register 				: Masks or unmasks events (used for wakeup or non-interrupt triggering). 	0 – Masked (Event disabled)			1 – Unmasked (Event enabled)						Address offset: 0x04
	__vo uint32_t RTSR;   //Rising Trigger Selection Register  	: Selects rising edge trigger detection.								 	0 – No trigger				     	1 – Rising edge trigger enabled						Address offset: 0x08
	__vo uint32_t FTSR;   //Falling Trigger Selection Register	: Selects falling edge trigger detection.								 	0 – No trigger				     	1 – Rising edge trigger enabled						Address offset: 0x0C
	__vo uint32_t SWIER;  //Software Interrupt Event Register	: Allows software to generate an interrupt (for testing or manual triggers) 0 – No effect		 				1 – Trigger interrupt								Address offset: 0x10
	__vo uint32_t PR;     //Pending Register					: Shows pending interrupts (indicates if an interrupt event occurred).   	0 – No pending interrupt			1 – Interrupt pending (must be cleared by writing 1 Address offset: 0x14

}EXTI_RegDef_t;

/*********************************************************SYSCFG************************************************/
typedef struct
{
	__vo uint32_t MEMRMP;       //Memory Remap Register				:             Address offset: 0x00      */
	__vo uint32_t PMC;          //		    									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    //External Interrupt Configuration Registers      Maps GPIO pins to EXTI lines for external interrupts.		:EXTICR1: 0x40013808 (EXTI0 – EXTI3), EXTICR2: 0x4001380C (EXTI4 – EXTI7), EXTICR3: 0x40013810 (EXTI8 – EXTI11), EXTICR4: 0x40013814 (EXTI12 – EXTI15)       Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  //			:         							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        //Compensation cell control register        	  Address offset: 0x20      */
} SYSCFG_RegDef_t;

/*********************************************************SPI************************************************/
typedef struct
{
	__vo uint32_t CR1;        //SPI Control Reg. 1    									Address offset: 0x00
	__vo uint32_t CR2;        //SPI Control Reg. 2										Address offset: 0x04
	__vo uint32_t SR;         //SPI Status Reg. 										Address offset: 0x08
	__vo uint32_t DR;         //SPI Data Reg.     										Address offset: 0x0C
	__vo uint32_t CRCPR;      //SPI CRC polynomial Reg.    								Address offset: 0x10
	__vo uint32_t RXCRCR;     //SPI RX CRC Reg.   										Address offset: 0x14
	__vo uint32_t TXCRCR;     //SPI TX CRC Reg.    										Address offset: 0x18
	__vo uint32_t I2SCFGR;    //SPI I2S Config Reg.    									Address offset: 0x1C
	__vo uint32_t I2SPR;      //SPI I2S Prescaler Reg.   								Address offset: 0x20
} SPI_RegDef_t;

/*********************************************************I2C************************************************/
typedef struct
{
  __vo uint32_t CR1;        /* Uses will be added later     										Address offset: 0x00 */
  __vo uint32_t CR2;        /* Uses will be added later     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /* Uses will be added later     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /* Uses will be added later     										Address offset: 0x0C */
  __vo uint32_t DR;         /* Uses will be added later     										Address offset: 0x10 */
  __vo uint32_t SR1;        /* Uses will be added later     										Address offset: 0x14 */
  __vo uint32_t SR2;        /* Uses will be added later     										Address offset: 0x18 */
  __vo uint32_t CCR;        /* Uses will be added later     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /* Uses will be added later     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /* Uses will be added later     										Address offset: 0x24 */
}I2C_RegDef_t;

/*********************************************************USART************************************************/
typedef struct
{
	__vo uint32_t SR;         /* uses  will be added later     										Address offset: 0x00 */
	__vo uint32_t DR;         /* uses  will be added later     										Address offset: 0x04 */
	__vo uint32_t BRR;        /* uses  will be added later     										Address offset: 0x08 */
	__vo uint32_t CR1;        /* uses  will be added later     										Address offset: 0x0C */
	__vo uint32_t CR2;        /* uses  will be added later     										Address offset: 0x10 */
	__vo uint32_t CR3;        /* uses  will be added later    										Address offset: 0x14 */
	__vo uint32_t GTPR;       /* uses  will be added later     										Address offset: 0x18 */
} USART_RegDef_t;




//peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
#define GPIOA  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  				((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4  				((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)





//----------------------------------Clock Enable Macros--------------------------------------------

//Clock Enable Macros for GPIOx peripherals
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))



//Clock Enable Macros for I2Cx peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


//Clock Enable Macros for SPIx peripheralsbu
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


//Clock Enable Macros for USARTx peripherals
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

//Clock Enable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


//Clock Disable Macros for GPIOx peripherals
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~(1 << 7))


//Clock Disable Macros for I2Cx peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))
//Clock Disable Macros for SPIx peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))
//Clock Disable Macros for USARTx peripherals
#define USART1_PCCK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCCK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCCK_DI() (RCC->APB1ENR &= ~(1 << 5))

//Clock Disable Macros for SYSCFG peripheral
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))


//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7:0)
/*
 * IRQ(Interrupt Request) Numbers of STM32F401RE MCU
 * NOTE: update these macros with valid values according to your MCU
*/

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART6	    71



//macros for all the possible priority levels
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15


//some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET




/******************************************************************************************
 *Clock Source Macros Value
 ******************************************************************************************/

#define HSE_VALUE   	8000000U
#define HSI_VALUE  		16000000U


/******************************************************************************************
 *Bit position definitions of RCCperipheral
 ******************************************************************************************/

//Macros for RCC clock control register (RCC_CR)
#define RCC_CR_PLLON 		24U
#define RCC_CR_PLLRDY 		25U

//Macros for RCC PLL configuration register (RCC_PLLCFGR)
#define RCC_PLLCFGR_PLLM 	0U
#define RCC_PLLCFGR_PLLN 	6U
#define RCC_PLLCFGR_PLLP 	16U
#define RCC_PLLCFGR_PLLQ 	24U
#define RCC_PLLCFGR_PLLSRC 	22U

//macros for RCC clock configuration register (RCC_CFGR)
#define RCC_CFGR_SW			0U
#define RCC_CFGR_SWS 		2U
#define RCC_CFGR_HPRE		4U
#define RCC_CFGR_PPRE1 		10U
#define RCC_CFGR_PPRE2 		13U
#define RCC_CFGR_MCO2PRE 	27U


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/

//Bit position definitions SPI_CR1
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

//Bit position definitions SPI_CR2
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

//Bit position definitions SPI_SR
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
//Bit position definitions I2C_CR1
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

//Bit position definitions I2C_CR2
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

//Bit position definitions I2C_OAR1
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

//Bit position definitions I2C_SR1
#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

//Bit position definitions I2C_SR2
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

//Bit position definitions I2C_CCR
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

//Bit position definitions USART_CR1
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

//Bit position definitions USART_CR2
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

//Bit position definitions USART_CR3
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

//Bit position definitions USART_SR
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f401re_gpio.h"
#include "stm32f401re_i2c.h"
#include "stm32f401re_usart.h"
#include "stm32f401re_rcc.h"


#ifdef __cplusplus
}
#endif

#endif /* STM32F401RE_H */
