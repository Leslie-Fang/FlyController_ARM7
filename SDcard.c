#include "SDcard.h"
#include "timer.h"
#include "spibaro.h"
#include "string.h"
void sd_init(void)
{
	*AT91C_PIOA_PER=SD_CS;		  	
	*AT91C_PIOA_OER=SD_CS;
	*AT91C_PIOA_SODR=SD_CS;
}
void sd_enable(void)
{
	*AT91C_PIOA_CODR=SD_CS;//low-active CSB
}
void sd_disable(void)
{
	*AT91C_PIOA_SODR=SD_CS;//disable CSB
}

unsigned char sd_Cmd0(void)
{	
	unsigned char ret,dummy[4];
	sd_enable();
	dummy[0]=SPI_B0_Send_byte(0x40);
	dummy[1]=SPI_B0_Send_byte(0x00);
	dummy[2]=SPI_B0_Send_byte(0x00);
	dummy[3]=SPI_B0_Send_byte(0x95);
	ret = SPI_B0_Receive_byte();
	dummy[0]=dummy[0];
	sd_disable();
	delay_ms(10);
	sd_enable();
	dummy[0]=SPI_B0_Send_byte(0x40);
	dummy[1]=SPI_B0_Send_byte(0x00);
	dummy[2]=SPI_B0_Send_byte(0x00);
	dummy[3]=SPI_B0_Send_byte(0x95);
	ret = SPI_B0_Receive_byte();
	dummy[0]=dummy[0];
	sd_disable();
	return ret;
}




