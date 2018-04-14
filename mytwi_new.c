#include "at91sam7s256.h"
#include "mytwi_new.h"
#include "timer.h"
#include "global.h"
void twi_init(void)
{
// 	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_TWI);		//TWI时钟使能
	*AT91C_PIOA_PDR|=0x18;		
	*AT91C_PIOA_ASR|=0x18;		//分配TWI
	*AT91C_PIOA_MDER|=0x18;		//pull-up
	*AT91C_TWI_IDR = 0x107;		
	*AT91C_TWI_CR =  AT91C_TWI_SWRST;
	*AT91C_TWI_CR =  AT91C_TWI_MSEN;
	//*AT91C_TWI_CWGR = 0x11920;
	*AT91C_TWI_CWGR = 0x11B1B;
	*AT91C_AIC_IDCR = (1<<AT91C_ID_TWI);		
}

void g_a_config()
{
	unsigned char a1[1] = {1};
	unsigned char a2[1] = {3};
	unsigned char a5[1] = {0};
	unsigned char a6[1] = {0x8};
	i2cwrite(MPU_ADDR,RA_PWR_MGMENT_1,1,a1);
	i2cwrite(MPU_ADDR,RA_SMPLRT_DIV,1,a2);
	i2cwrite(MPU_ADDR,RA_CONFIG,1,a2);
	i2cwrite(MPU_ADDR,RA_GYRO_CONFIG,1,a5);
	i2cwrite(MPU_ADDR,RA_ACCEL_CONFIG,1,a6);
	i2cwrite(MPU_ADDR,RA_USER_CTRL,1,a5);
	i2cwrite(MPU_ADDR,RA_INT_PIN_CFG,1,a5);
	i2cwrite(MPU_ADDR,RA_INT_ENABLE,1,a5);
}

void compass_config()
{
	unsigned char data_write[3];
	data_write[0]=0x02;
	data_write[1]=0x00;
	data_write[2]=0x01;

	i2cwrite(MPU_ADDR,RA_BYPASS_CFG,1,data_write);
	delay_ms(10);
	i2cwrite(MPU_ADDR,RA_USER_CTRL,1,(data_write+1));
	delay_ms(10);
	i2cwrite(COMPASS_ADDR,COMPASS_CTRL,1,(data_write+2));

}

unsigned char i2cwtritebyte(unsigned char address, unsigned short reg, unsigned char *data)
{
	*AT91C_TWI_CR =  AT91C_TWI_MSEN; 
    *AT91C_TWI_MMR = ( (address<<16) | AT91C_TWI_IADRSZ_1_BYTE ) & ~AT91C_TWI_MREAD;            
    *AT91C_TWI_IADR = reg;             
    *AT91C_TWI_THR = *data;       
   	while (!(*AT91C_TWI_SR & AT91C_TWI_TXRDY));   
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));              
    return AT91C_EEPROM_WRITE_OK;
}

unsigned char i2cwrite(unsigned char address, unsigned short reg, unsigned char len, unsigned char *data)
{
	int i;
	int k;
	for(i=0;i<len;i++){
		k=i2cwtritebyte(address,reg,(data+i));	
	}
	return k;	
}

unsigned char i2creadbyte(unsigned char address, unsigned short reg, char *buf)
{
	*AT91C_TWI_CR = 0x00000004;
    *AT91C_TWI_MMR = (address<<16) | AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;       
    *AT91C_TWI_IADR = reg;   
    *AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;   
    while (!(*AT91C_TWI_SR & AT91C_TWI_TXCOMP));            
    *buf = *AT91C_TWI_RHR;              
    return AT91C_EEPROM_READ_OK; 
}
int twi_read(const AT91PS_TWI pTwi , int intAddress, char *data, int size)   
{    		   
	*AT91C_TWI_CR = 0x00000004;
    pTwi->TWI_MMR = IMU_ADD | size | AT91C_TWI_MREAD;       
    pTwi->TWI_IADR = intAddress;   
    pTwi->TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;   
    while (!(pTwi->TWI_SR & AT91C_TWI_TXCOMP));            
    *data = pTwi->TWI_RHR;              
    return AT91C_EEPROM_READ_OK;   
}

unsigned char i2cread(unsigned char address, unsigned short reg, unsigned char len, char *buf)
{
	int i;
	int k;
	for(i=0;i<len;i++){
		k=i2creadbyte(address,reg,(buf+i));	
	}
	return k;	
}

int twi_write(const AT91PS_TWI pTwi, int intAddress, char *data2send, int size)
{ 
	*AT91C_TWI_CR =  AT91C_TWI_MSEN; 
    pTwi->TWI_MMR = ( IMU_ADD | size ) & ~AT91C_TWI_MREAD;            
    pTwi->TWI_IADR = intAddress;             
    pTwi->TWI_THR = *data2send;       
   	while (!(pTwi->TWI_SR & AT91C_TWI_TXRDY));   
    while (!(pTwi->TWI_SR & AT91C_TWI_TXCOMP));              
    return AT91C_EEPROM_WRITE_OK;
}



void twi_read_all(char *buffer){
	int i;
	for (i=0; i<6; i++){
	 	twi_read(AT91C_BASE_TWI, (0x3B+i), &buffer[i], AT91C_TWI_IADRSZ_1_BYTE);
	}
	for (i=0; i<6; i++){
		twi_read(AT91C_BASE_TWI, (0x43+i), &buffer[6+i], AT91C_TWI_IADRSZ_1_BYTE);
	}
}
int twi_readcompass(char *buffer)
{
	unsigned char data[1],i;
	char tmp[1];
	data[0]=0x01;
	tmp[0]=0x00;
	i2cread(COMPASS_ADDR,COMPASS_ST1,1,tmp);
	if(tmp[0]==1){
		 for(i=0;i<6;i++){
		 	i2cread(COMPASS_ADDR,(COMPASS_HXL+i),1,&buffer[i]);
		 }
		 i2cwrite(COMPASS_ADDR,COMPASS_CTRL,1,data);
		 return 1;
	}
	return 0;
}

void compass_init_read()
{
	unsigned char data[1],i;
	char data_rcv[6];
	char tmp[1];
	data[0]=0x01;
	tmp[0]=0x00;
	i2cwrite(COMPASS_ADDR,COMPASS_CTRL,1,data);
	delay_ms(10);
	i2cread(COMPASS_ADDR,COMPASS_ST1,1,tmp);
	if(tmp[0]==1){
		 for(i=0;i<6;i++){
		 	i2cread(COMPASS_ADDR,(COMPASS_HXL+i),1,&data_rcv[i]);
		 }
		 i2cwrite(COMPASS_ADDR,COMPASS_CTRL,1,data);
	}
}

