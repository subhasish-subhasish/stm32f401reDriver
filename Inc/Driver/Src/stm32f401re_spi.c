#include "stm32f401re_spi.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}else if(pSPIx==SPI3)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx==SPI3)
	    {
			SPI3_PCLK_DI();
		}else if(pSPI==SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle){
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//First let config the spi CR1 Reg
	uint32_t tempreg=0;  //stores all the conig value in ths temp then transfer it to the CR1 reg
	//1.config device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//2.config bus (Half dulplex , FUll duplex, RX only
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be clear
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mofe should be clear
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//rx only bit must be set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}
	//3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF (DATA frame Format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//1. wait until TXE is set  TXE:0 - TXE buff is not empty | TXE:1 - TXE buff is empty
		while( SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

		//2. check the dff bit
		if((pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR =   *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit data
			pSPIx->DR =*pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//0: Rx buffer empty | 1: Rx buffer not empty
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);
		if((pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ))
		{
			//16 bit
			*((uint16_t*)pRxBuffer) =pSPIx->DR ;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++; //type cated to uint16_t pointer from uint8_t pointer
		}else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR ;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31 )
		{
			*NVIC_ISER0 |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}else{
		if(IRQNumber <= 31 )
		{
			*NVIC_ICER0 |= (1<<IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			*NVIC_ICER1 |= (1<<(IRQNumber % 32));
		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1<<(IRQNumber % 64));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQPriority %  4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state =pSPIHandle->RxState;

	if(state!= SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	//lets check for txe
	temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 &(1<<SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}
	temp1 = pHandle->pSPIx->SR &(1<<SPI_SR_RXNE);
	temp1 = pHandle->pSPIx->CR2 &(1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		spi_rxe_interrupt_handle(pHandle);
	}
	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle){
		//2. check the dff bit
		if((pHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ))
		{
			//16 bit DFF
			//1. load the data in to the DR
			pHandle->pSPIx->DR =   *((uint16_t*)pHandle->pTxBuffer);
			pHandle->TxLen --;
			pHandle->TxLen --;
			(uint16_t*)pHandle->pTxBuffer++;
		}else
		{
			//8 bit data
			pHandle->pSPIx->DR = *pHandle->pTxBuffer;
			pHandle->TxLen --;
			pHandle->pTxBuffer++;
		}
		if(! pHandle->TxLen)
		{
			//TxLen is zero , so close the spi transmission and inform the application that
			//TX is over.

			//this prevents interrupts from setting up of TXE flag
			SPI_CloseTransmisson(pHandle);
			SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
		}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pHandle)
{
	if(pHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		pHandle->pSPIx->DR = *((uint16_t*)pHandle->pRxBuffer);
		pHandle->RxLen--;
		pHandle->RxLen--;
		(uint16_t*)pHandle->pRxBuffer++;
	}else
	{
		//8 bit data
		pHandle->pSPIx->DR = *pHandle->pRxBuffer;
		pHandle->RxLen --;
		pHandle->pRxBuffer++;
	}
	if(! pHandle->RxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.
		//this prevents interrupts from setting up of TXE flag
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}



void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}


void  SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}

}
