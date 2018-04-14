#include "EEPROM.h"
#include "at91sam7s256.h"
#include "mytwi_new.h"
#include "timer.h"
#include "global.h"
/*
	eeprom_read(_data1, 18, PAGE_1, 0);
	eeprom_read(_data2, 18, PAGE_2, 0);
//	eeprom_write(data1, 18, PAGE_1, 0);
//	eeprom_write(data2, 18, PAGE_2, 0);
	eeprom_read(_data1, 18, PAGE_2, 0);
	eeprom_read(_data2, 18, PAGE_1, 0);		
*/
int eeprom_write(
	void *data, 
	unsigned char len, 
	unsigned short page, 
	unsigned char word)
{
	unsigned char i;
	for(i=0;i<len;i++){
		i2c_eeprom_write_byte(EEPROM_ADDRESS, page+word+i, ((unsigned char *)data+i));
		delay_ms(10);
	}
	return 1;
}

int eeprom_read(
	char *buffer, 
	unsigned char len, 
	unsigned short page, 
	unsigned char word)
{
	char i;
	for(i=0;i<len;i++){
		i2c_eeprom_read_byte(EEPROM_ADDRESS, page+word+i, (buffer+i));
	}
	return 1;
}
unsigned char i2c_eeprom_write_byte(unsigned char address, unsigned short reg, unsigned char *data)
{
	*AT91C_TWI_CR =  AT91C_TWI_MSEN; 
    *AT91C_TWI_MMR = ( (address<<16) | AT91C_TWI_IADRSZ_2_BYTE ) & ~AT91C_TWI_MREAD;            
    *AT91C_TWI_IADR = reg;             
    *AT91C_TWI_THR = *data;       
   	while (!(*AT91C_TWI_SR & AT91C_TWI_TXRDY));
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));  
    return AT91C_EEPROM_WRITE_OK;
}
unsigned char i2c_eeprom_read_byte(unsigned char address, unsigned short reg, char *buf)
{
	*AT91C_TWI_CR = 0x00000004;
    *AT91C_TWI_MMR = (address<<16) | AT91C_TWI_IADRSZ_2_BYTE | AT91C_TWI_MREAD;       
    *AT91C_TWI_IADR = reg;   
	*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;   
    while (!(*AT91C_TWI_SR & AT91C_TWI_RXRDY));            
    *buf = *AT91C_TWI_RHR;   
	while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP)); 
    return AT91C_EEPROM_READ_OK; 
}

