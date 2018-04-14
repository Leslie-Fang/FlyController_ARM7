#include "at91sam7s256.h"
#include "mypit.h"
#include "main.h"
#include "timer.h"
#include "global.h"
#if ORIGINAL_FREQ
#define PIV (3000000/PIT_FREQ-1)//599//59
#elif DOUBLED_FREQ
#define PIV (6000000/PIT_FREQ-1)
#endif

__irq void pit_int_handler(void){
//	static int cntSonar;
//	short time=0;
//	static int Period_20ms=0;
//	unsigned int CurrentPins=0;
//	unsigned int Mask=0; 	
//	static unsigned int LastPins;

//	if (*AT91C_PIOA_PDSR & IN5){CurrentPins|=1;}
 /*
	Mask=CurrentPins ^ LastPins;
	LastPins=CurrentPins;
	if(Mask){
		if(CurrentPins){//上升沿
			cntSonar=0;
		}
		else{//下降沿
			time=cntSonar;//记录高电平时间
			if(cmd.SonarEnable)
				pos.sonarPos=(float)time*17/2500;//cm unit			
		}
	}
	cntSonar++;

//	for(i=4;i<5;i++){pin[i].TimeCount++;}


	if(Period_20ms==1000){
		Period_20ms=0;
		if(cmd.SonarEnable)
			*AT91C_PIOA_SODR|=SONAR_TRIG;
	}
	if(Period_20ms>=2){
		*AT91C_PIOA_CODR|=SONAR_TRIG;
	}
	Period_20ms++;
*/
//	smpl.Count500Hz++;
//	smpl.UARTreceiveCount++;
//	if (smpl.Count500Hz == 10){
//		smpl.Count500Hz =0;
//		smpl.Flag500Hz = 1;
//	}	
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_SYS); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR; /* Ack & End of Interrupt */
}

void pit_init () 
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PITC_PIMR = AT91C_PITC_PITIEN | AT91C_PITC_PITEN | (PIV);	

	pAIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_SYS] = (unsigned long) pit_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_SYS); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_SYS); 
	
//	    *AT91C_PMC_PCER |= (1<<AT91C_ID_PIOA);
//	*AT91C_PIOA_PER |= PWMIN_MASK;	  
//	*AT91C_PIOA_ODR |= PWMIN_MASK;
//	*AT91C_PIOA_PPUDR |= PWMIN_MASK;
//	*AT91C_PIOA_PER |= SONAR_TRIG;	  	
//	*AT91C_PIOA_OER |= SONAR_TRIG;
	*AT91C_PMC_PCER |= (1<<AT91C_ID_PIOA);
	pAIC->AIC_SMR[AT91C_ID_PIOA] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 7;
	pAIC->AIC_SVR[AT91C_ID_PIOA] = (unsigned long)PIO_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_PIOA); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_PIOA);
	*AT91C_PIOA_PER |= IN0;
	*AT91C_PIOA_ODR |= IN0;
//	*AT91C_PIOA_IFER |= IN0;
//	*AT91C_PIOA_CODR |= IN0;
	*AT91C_PIOA_IDR = 0xFFFFFFFF;
//	*AT91C_PIOA_IER = IN0;
	
	*AT91C_PIOA_MDDR |= IN0;
	*AT91C_PIOA_PPUDR |= IN0;
	*AT91C_PIOA_ASR &= ~(IN0);
	*AT91C_PIOA_BSR &= ~(IN0);
	*AT91C_PIOA_OWDR |= IN0;
	*AT91C_PIOA_ISR;
	*AT91C_PIOA_IER = IN0;
}

__irq void PIO_handler(void){
	int timePPM=0;
	int status;
	static unsigned short channel=0;
	status = *AT91C_PIOA_ISR;
    status &= *AT91C_PIOA_IMR;
//	if(status & USB_VBUS){
	//	if (*AT91C_PIOA_PDSR & USB_VBUS)
	//		myusb.connect_flag = PLUG_IN;
	//	else
	//		myusb.connect_flag = PLUG_OUT;
//	}
//	else 
	if(status & IN0){
		if (*AT91C_PIOA_PDSR & IN0){
			timePPM = ppm_get_time();
			if(timePPM>=0 && timePPM<=4000){
				if(channel < 9){
				#if F450
					cmd.rc[channel]=244*(timePPM-1220)/100;
				#elif XINSONG
					cmd.rc[channel]=244*(timePPM-1220)/100;
				#elif F330
					cmd.rc[channel]=242*(timePPM-1222)/100;
				#elif F240
					cmd.rc[channel]=-1024;
				#endif
				}
				channel++;				
			} 
			else{
				channel=0;
			}		
		}
		else{
			ppm_reset_clock();
		}
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_PIOA);
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR;
	return;
}
