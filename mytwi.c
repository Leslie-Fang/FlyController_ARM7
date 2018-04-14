#include "at91sam7s256.h"
#include "mytwi.h"
#define PIT_PIV_MICRO_SECOND_VALUE  0x2

int TWI_TX_Comp = 0;
int TWI_RX_Ready = 0;
int TWI_TX_Ready = 0;
int TWI_Ack = AT91C_TWI_NACK;
char st[1]={1};


void twi_configure()
{
	*AT91C_PMC_PCER|=(1<<AT91C_ID_TWI);		//TWI时钟使能
	*AT91C_PIOA_PDR|=0x18;		//Disable I/O,PA3, PA4
	*AT91C_PIOA_ASR|=0x18;		//分配SPI
	*AT91C_PIOA_MDER|=0x18;		//分配SPI
	*AT91C_TWI_IDR = 0x107;		//disable interrupt
	*AT91C_TWI_CR =  AT91C_TWI_SWRST;
//	*AT91C_TWI_CR =  AT91C_TWI_MSEN;
	*AT91C_TWI_CWGR = 0x2EFEF;
	*AT91C_AIC_IDCR = (1<<AT91C_ID_TWI);

}

void AT91F_TWI_WaitMicroSecond (unsigned int MicroSecond)   
{   
   unsigned int PitStatus = 0;     /* Status register of the PIT */   
   unsigned int PitLoop = 0;    /* Store the number of PIT Loop */   
   
   AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN|PIT_PIV_MICRO_SECOND_VALUE;   
   
   for( PitLoop=0; PitLoop <(MicroSecond);)   /* One PIT loop equals 333ns */   
   {   
    /* Wait for the PIT counter overflow occurs */   
    while ((AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS)==0);   
    /* Read the PIT Interval Value Reg. to clear it for the next overflow */   
    PitStatus = AT91C_BASE_PITC->PITC_PIVR ;   
    /* dummy access to avoid IAR warning */   
    PitStatus = PitStatus ;   
    PitLoop++;   
   }   
}  
int AT91F_TWI_ReadSingleIadr(const AT91PS_TWI pTwi,   int SlaveAddr,   int IntAddr,   int IntAddrSize,   char *data)   
{   
    unsigned int status,error=0, end=0;   
   	
    /* Enable Master Mode */   
    pTwi->TWI_CR = AT91C_TWI_MSEN ;   
   
    /* Set the TWI Master Mode Register */   
    pTwi->TWI_MMR =  SlaveAddr | (IntAddrSize) | AT91C_TWI_MREAD;   
   
    /* Set TWI Internal Address Register if needed */   
    pTwi->TWI_IADR = IntAddr;   
   
    pTwi->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;   
   
    /* NACK errata handling */   
    /* Do not poll the TWI_SR */   
    /* Wait 3 x 9 TWCK pulse (max) 2 if IADRR not used, before reading TWI_SR */   
    /* From 400Khz down to 1Khz, the time to wait will be in 祍 range.*/   
    /* In this example the TWI period is 1/400KHz */   
    AT91F_TWI_WaitMicroSecond (100) ;   
   
    while (!end)   
    {   
      status = AT91C_BASE_TWI->TWI_SR;   
      if ((status & AT91C_TWI_NACK) == AT91C_TWI_NACK)   
      {   
        error++;   
        end=1;   
      }   
    /*  Wait for the receive ready is set */   
      if ((status & AT91C_TWI_RXRDY) == AT91C_TWI_RXRDY)   
        end=1;   
    }   
   
    *(data) = pTwi->TWI_RHR;   
   	
	//for (; i > 0; i--);
   /* Wait for the Transmit complete is set */   
   status = AT91C_BASE_TWI->TWI_SR;   
   while (!(status & AT91C_TWI_TXCOMP))   
     status = AT91C_BASE_TWI->TWI_SR;   
   
    return 0;   
}   
   
int AT91F_TWI_WriteSingleIadr(const AT91PS_TWI pTwi,   int SlaveAddr,   int IntAddr,   int IntAddrSize)   
{   
    unsigned int end = 0, status, error=0;   
   
    /* Enable Master Mode */   
    pTwi->TWI_CR = AT91C_TWI_MSEN ;   
   
    /* Set the TWI Master Mode Register */   
    pTwi->TWI_MMR =  SlaveAddr | (IntAddrSize) & ~AT91C_TWI_MREAD;   
   
    /* Set TWI Internal Address Register if needed */   
    pTwi->TWI_IADR = IntAddr;   
   
    /* Write the data to send into THR. Start conditionn DADDR and R/W bit  
       are sent automatically */   
    pTwi->TWI_THR = 0x01;   
   
    /* NACK errata handling */   
    /* Do not poll the TWI_SR */   
    /* Wait 3 x 9 TWCK pulse (max) 2 if IADRR not used, before reading TWI_SR */   
    /* From 400Khz down to 1Khz, the time to wait will be in 祍 range.*/   
    /* In this example the TWI period is 1/400KHz */   
    AT91F_TWI_WaitMicroSecond (100) ;   
   
    while (!end)   
    {   
      status = AT91C_BASE_TWI->TWI_SR;   
      if ((status & AT91C_TWI_NACK) == AT91C_TWI_NACK)   
      {   
        error++;   
        end=1;   
      }   
    /*  Wait for the Transmit ready is set */   
      if ((status & AT91C_TWI_TXRDY) == AT91C_TWI_TXRDY)   
        end=1;   
    }   
   
    /* Wait for the Transmit complete is set */   
    status = AT91C_BASE_TWI->TWI_SR;   
    while (!(status & AT91C_TWI_TXCOMP))   
      status = AT91C_BASE_TWI->TWI_SR;   
   
    return error;   
}				 
