#include "twi_fast.h"
#include "LED.h"
#include "at91sam7s256.h"
#include "IMU.h"
#include "global.h"

TWI_Reader readers[3]={
//acc
	{0,{0x3B,0x3C,0x3D,0x3E,0x3F,0x40},0x68,0},
//gyro
	{0,{0x43,0x44,0x45,0x46,0x47,0x48},0x68,0},
//compass
	{0,{0x03,0x04,0x05,0x06,0x07,0x08},0x1E,0}
};	
char stop_signal=0;
//set at stop_twi_after_complete(),cleared at last compass interrupt
char read_rdy_flag=1;
//set at stop_twi_now() and last compass interrupt,
//cleared at twi_g_a_read_start() and twi_cps_read_start()
char working_reader=0;
//set as G_A_SWITCH at twi_g_a_read_start(),
//set as CPS_SWITCH at twi_cps_read_start(),
//set as NON_WORKING at stop_twi_now() and last compass interrupt

	
void twi_fast_init(void)
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
//	*AT91C_PMC_PCER|=(1<<AT91C_ID_TWI);		//TWI时钟使能
	pAIC->AIC_SMR[AT91C_ID_TWI] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_TWI] = (unsigned long)twi_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TWI); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TWI);
//	*AT91C_PIOA_PDR|=0x18;		
//	*AT91C_PIOA_ASR|=0x18;		//分配TWI
//	*AT91C_PIOA_MDER|=0x18;		//pull-up
	*AT91C_TWI_IDR = 0x107;		
//	*AT91C_TWI_CR =  AT91C_TWI_SWRST;
//	*AT91C_TWI_CR =  AT91C_TWI_MSEN;
//	*AT91C_TWI_CWGR = 0x11920;
//	*AT91C_TWI_IER = AT91C_TWI_TXCOMP;	
	continue_acc_read();
}
int twi_gyro_read_start(char *buf)
{	
	readers[GYRO_SWITCH].Buf = buf;
//	readers[GYRO_SWITCH].sizeRemain = 6;
//	readers[GYRO_SWITCH].intAdd = INT_ADD_G_A;
//	readers[GYRO_SWITCH].devAdd = DEV_ADD_G_A;
	readers[GYRO_SWITCH].readCnt = 0;
	
	*AT91C_TWI_CR = AT91C_TWI_MSEN;
	*AT91C_TWI_MMR = (readers[GYRO_SWITCH].devAdd << 16) 
		| AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;
	*AT91C_TWI_IADR = readers[GYRO_SWITCH].intAdd[0];
//	if(read_rdy_flag){
		*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
		*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
		working_reader = GYRO_SWITCH;
		read_rdy_flag = 0;
		return 0;
//	}
//	else{//read not ready
//		return READ_START_NOT_RDY;
//	}
}
int twi_acc_read_start(char *buf)
{
	readers[ACC_SWITCH].Buf = buf;
//	readers[ACC_SWITCH].sizeRemain = 6;
//	readers[ACC_SWITCH].intAdd = INT_ADD_CPS;
//	readers[ACC_SWITCH].devAdd = DEV_ADD_CPS;
	readers[ACC_SWITCH].readCnt = 0;	
	*AT91C_TWI_CR = AT91C_TWI_MSEN;
	*AT91C_TWI_MMR = (readers[ACC_SWITCH].devAdd << 16) 
		| AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;
	*AT91C_TWI_IADR = readers[ACC_SWITCH].intAdd[0];
//	if(read_rdy_flag){
		*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
		*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
		working_reader = ACC_SWITCH;
		read_rdy_flag = 0;
		return 0;
}
int twi_cps_read_start(char *buf)
{
	readers[CPS_SWITCH].Buf = buf;
//	readers[CPS_SWITCH].sizeRemain = 6;
//	readers[CPS_SWITCH].intAdd = INT_ADD_CPS;
//	readers[CPS_SWITCH].devAdd = DEV_ADD_CPS;
	readers[CPS_SWITCH].readCnt = 0;	
//	*AT91C_TWI_CR = AT91C_TWI_MSEN;
	*AT91C_TWI_MMR = (readers[CPS_SWITCH].devAdd << 16) 
		| AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;
	*AT91C_TWI_IADR = readers[CPS_SWITCH].intAdd[0];
//	if(read_rdy_flag){
		*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
		*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
		working_reader = CPS_SWITCH;
		read_rdy_flag = 0;
		return 0;
//	}
//	else{//read not ready
//		return READ_START_NOT_RDY;
//	}
}
void stop_twi_now()
{
	working_reader = NON_WORKING;
	read_rdy_flag = 1;
	*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
}
void stop_twi_after_complete()
{
	stop_signal = 1;
}
__irq void twi_int_handler(void)
{
	int status;
	static short cps_cnt = 0;
	status = *AT91C_TWI_IMR;
	status = *AT91C_TWI_SR;
	if(status & AT91C_TWI_TXCOMP){
		if(working_reader !=NON_WORKING){
			*((readers[working_reader].Buf)+readers[working_reader].readCnt) = *AT91C_TWI_RHR;
			readers[working_reader].readCnt++;
			if(readers[working_reader].readCnt < 6){
				*AT91C_TWI_IADR = readers[working_reader].intAdd[readers[working_reader].readCnt];
				*AT91C_TWI_CR = AT91C_TWI_START|AT91C_TWI_STOP;
			}
			else{				
				if(working_reader==ACC_SWITCH){//a over, next g
					data_conclude(ACC_SWITCH);
					continue_gyro_read();
					acc_lowpass();				
				}
				else if(working_reader==GYRO_SWITCH){//g over, next cps
					data_conclude(GYRO_SWITCH);
					cps_cnt++;
					if(cps_cnt > 7){//75Hz, 13.33ms, 7*2ms = 14ms
						cps_cnt = 0;
						continue_cps_read();
					}else{
						working_reader = NON_WORKING;
						read_rdy_flag = 1;
						*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
					}				
					smpl.Flag500Hz = 1;	
				}
				else if(working_reader==CPS_SWITCH){//cps over		
					data_conclude(CPS_SWITCH);
					sens.mag_updated = 1;		
				//	if(stop_signal){
				//		stop_signal=0;
						working_reader = NON_WORKING;
						read_rdy_flag = 1;
						*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
				}				
			}
		}
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TWI);
	*AT91C_AIC_EOICR = 0x1;
}
