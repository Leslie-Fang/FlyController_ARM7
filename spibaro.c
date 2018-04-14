#include "at91sam7s256.h"
#include "spibaro.h"
#include "timer.h"	
#include "math.h"
#include "main.h"
#include "global.h"
#include "PosEst.h"
unsigned short setup,CRC,C1,C2,C3,C4,C5,C6;
unsigned long D1,D2; 
long TEMP;
long long OFF,SENS,dT;
long long OFF2,SENS2,T2,AUX,AUX2;
void data2tempeture()
{  	
	dT = (long long)D2 - ((unsigned long)C5 << 8);
  	TEMP = 2000 + ((C6 * dT) >> 23);
	OFF = ((unsigned long)C2 << 16) + ((C4 * dT) >> 7);
	SENS = ((unsigned long)C1 << 15) + ((C3 * dT) >> 8);
	if(TEMP < 2000){
		T2 = (dT * dT) >> 31;
		AUX = (TEMP - 2000) * (TEMP - 2000);
		OFF2 = 5 * AUX >> 1;
		SENS2 = 5 * AUX >> 2;
		if(TEMP < -1500){
			AUX2 = (TEMP + 1500) * (TEMP + 1500);
			OFF2 += 7 * AUX2;
			SENS2 += 11 * AUX2 / 2;
		}
	}
	else{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;	
	baro.temp = TEMP;
}
void data2pressure()
{	
	baro.pressure = ((D1 * SENS >> 21) - OFF) >> 15;
}
float get_altitude()
{
	float tmp_float, altitude;
	float refPressure=baro.refPressure;
//  tmp_float = (Pressure / 101325.0);
	tmp_float = ((float)baro.pressure / refPressure);
	tmp_float = pow(tmp_float, 0.190295);
	altitude = 44330000 * (1.0 - tmp_float);
	return altitude;
}
void spi_init(void)
{	   
	*AT91C_PMC_PCER|=(1<<AT91C_ID_SPI);
	*AT91C_PIOA_PDR|=0x7000;	//??4???????????
	*AT91C_PIOA_ASR|=0x7000;//PA12,13,14,?????A,PA11??????????
	*AT91C_SPI_CR|=AT91C_SPI_SPIEN;
	*AT91C_SPI_MR=AT91C_SPI_MSTR|AT91C_SPI_PS_FIXED|AT91C_SPI_MODFDIS|0xe0000;
	//????:?????????????????,?????NPCS?????NPCS?????????
	*AT91C_SPI_IDR|=0x03FF;//all disabled	
    *AT91C_SPI_CSR|=(0x0<<24)|(0x0<<16)|(0x92 << 8)|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;
	*AT91C_AIC_IDCR = (1<<AT91C_ID_SPI);	
	*AT91C_PIOA_PER=1<<11;		  	
	*AT91C_PIOA_OER=1<<11;
	*AT91C_PIOA_CODR=1<<11;//low-active CSB
		 //???????CSB???????????		
	MS5611_RESET();
	delay_ms(10);
	*AT91C_PIOA_SODR=1<<11;
	delay_ms(10);
//	MS5611_PROM_READ();	
//	delay_ms(10);
}
void pressure_process(short step)
{
	 unsigned char PH,PM,PL;
	 unsigned char dummy=0;
	 float rawAlt;
	 switch (step){
	 case 1:	 		
		SPIenable();
		sendorder(CMD_CONVERT_D1_OSR4096);
		dummy=SPI_getdata();
		SPIdisable();
		break;
	case BARO_STEPS:
		SPIenable();
		sendorder(MS5611_ADC);
		dummy=SPI_getdata();
		send0data();
		PH=SPI_getdata();
		send0data();
		PM=SPI_getdata();
		send0data();
		PL=SPI_getdata();
		SPIdisable();
		D1=(((unsigned long)PH)<<16) | (((unsigned long)PM)<<8) | ((unsigned long)PL);
		data2pressure();
		if(baro.refPressure < 120000 && baro.refPressure > 80000){
			rawAlt=get_altitude();
			baro.alt = rawAlt;//kalman_filter(rawAlt,0.5,1.0,BARO_FILT);
		}				
	 	break;
	 default:
	 	break;
	 }
	 dummy=dummy;
}
void temperature_process(short step)
{
	unsigned char TH,TM,TL;
	unsigned char dummy=0;
	switch (step){
	case 1:
		SPIenable();
		sendorder(CMD_CONVERT_D2_OSR4096);		 
		dummy=SPI_getdata();
		SPIdisable(); //end of strobe
		break;
	case BARO_STEPS:
   		SPIenable();
		sendorder(MS5611_ADC);
		dummy=SPI_getdata();
		send0data();
		TH=SPI_getdata();
		send0data();
		TM=SPI_getdata();
		send0data();
		TL=SPI_getdata();
		SPIdisable();
		D2=(((unsigned long)TH)<<16) | (((unsigned long)TM)<<8) | ((unsigned long)TL);
		data2tempeture();
		break;
	default:
	 	break;
	 }
	 dummy=dummy;
}
void MS5611_getTemperature(void)
{
	SPIenable();
	SPI_B0_Strobe(CMD_CONVERT_D2_OSR4096);
	D2=MS5611_SPI_read_ADC(); 
	SPIdisable();
	data2tempeture();
}
void MS5611_getPressure(void)
{
	SPIenable();
	SPI_B0_Strobe(CMD_CONVERT_D1_OSR4096);
	D1=MS5611_SPI_read_ADC();
	SPIdisable();
	data2pressure();
}
unsigned char SPI_B0_Send_byte(unsigned char byte)
{    
	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=byte;  
  	delay_ms(10);//must be long enough
  	while (!((*AT91C_SPI_SR) & AT91C_SPI_RDRF)); 
	delay_ms(10); 
  	return(*AT91C_SPI_RDR);
	
}
unsigned char SPI_B0_Receive_byte(void)
{ 
	unsigned char data;
	data=SPI_B0_Send_byte(0);
	return data;
}
unsigned long MS5611_SPI_read_ADC()
{
  	unsigned char byteH,byteM,byteL;
  	unsigned long return_value;
  	SPIenable(); 
    SPI_B0_Send_byte(MS5611_ADC);
  	byteH = SPI_B0_Receive_byte();
  	byteM = SPI_B0_Receive_byte();
  	byteL = SPI_B0_Receive_byte();
  	SPIdisable();
  	return_value = (((long)byteH)<<16) | (((long)byteM)<<8) | (byteL);
  	return(return_value);
}
void SPI_B0_Strobe(unsigned char strobe)
{ 	
	SPIenable();
  	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE)); //???????????
   	SPI_B0_Send_byte(strobe);             // Send strobe
   	SPIdisable();
}
void sendorder(unsigned char order)
{	               
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=order; 
}
void send0data(void)
{                    
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=0;
}
unsigned char SPI_getdata()
{
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_RDRF));
	return(*AT91C_SPI_RDR);
}
unsigned short SPI_B0_Read_16bits(unsigned char addr)
{
  	unsigned char byteH,byteL;
  	unsigned short return_value;
   	SPIenable();
	SPI_B0_Send_byte(addr);
	byteH = SPI_B0_Receive_byte();
	delay_ms(1);
	byteL = SPI_B0_Receive_byte();
	SPIdisable();
	return_value = (((unsigned short)byteH)<<8) | (byteL);
	return(return_value);    
}
void MS5611_PROM_READ()
{
  	short data3[9];
  	SPIenable();
  	C1 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C1);
  	C2 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C2);
  	C3 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C3);
  	C4 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C4);
  	C5 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C5);
  	C6 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C6);
	C1 = SPI_B0_Read_16bits(CMD_MS5611_PROM_C1);
  	data3[6] = SPI_B0_Read_16bits(CMD_MS5611_RESET);
  	data3[7] = SPI_B0_Read_16bits(CMD_MS5611_PROM_CRC);
	data3[0]=data3[0];
	SPIdisable();
}
void SPIenable(void)
{
	*AT91C_PIOA_CODR=1<<11;//low-active CSB
}
void SPIdisable(void)
{
	*AT91C_PIOA_SODR=1<<11;//disable CSB
}
void MS5611_RESET(void)
{		
//	SPIenable();
	sendorder(CMD_MS5611_RESET);
//	SPIdisable();
}
