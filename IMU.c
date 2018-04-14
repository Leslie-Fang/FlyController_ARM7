#include "global.h"
#include "IMU.h"
#include "timer.h"
#include "mytwi_new.h"
#include "LED.h"
#include "twi_fast.h"
#include "Filter.h"
#include "compass_on_gps.h"
IIRFilter iir_ax={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_ay={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_az={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
char IMU_received[12];
char compass_received[6];
short maxMagX=0,maxMagY=0,maxMagZ=0;
short minMagX=0,minMagY=0,minMagZ=0;
char testing;
short gyr_x_bias=0;
short gyr_y_bias=0;
short gyr_z_bias=0;
short ax,ay,az;
#if BOARD_V5
	#if F330
	short acc_x_bias=0;
	short acc_y_bias=0;
	short acc_z_bias=-250;
		#if MPU_COMPASS
		short mag_x_bias=0;
		short mag_y_bias=0;
		short mag_z_bias=0;
		#elif HMC_COMPASS
		short mag_x_bias=-30;
		short mag_y_bias=-30;
		short mag_z_bias=-35;
		#endif
	#elif F450
	short acc_x_bias=10;
	short acc_y_bias=0;
	short acc_z_bias=100;
		#if MPU_COMPASS
		short mag_x_bias=0;
		short mag_y_bias=0;
		short mag_z_bias=0;
		#elif HMC_COMPASS
		short mag_x_bias=-150;
		short mag_y_bias=30;
		short mag_z_bias=20;
		#endif
	#elif XINSONG
		short acc_x_bias=-50;
		short acc_y_bias=140;
		short acc_z_bias=-366;
		#if MPU_COMPASS
		short mag_x_bias=0;
		short mag_y_bias=0;
		short mag_z_bias=0;
		#elif HMC_COMPASS
		short mag_x_bias=-20;
		short mag_y_bias=30;
		short mag_z_bias=-160;
		#endif
	#endif
#endif



void compass_calibration_xy(void)
{
	while(1){//wait for init calibrating
		if(testing){
			mag_x_bias=0;
			mag_y_bias=0;
			maxMagX=0,maxMagY=0,maxMagZ=0;
			minMagX=0,minMagY=0,minMagZ=0;
			while(testing){
				get_compass_bias();
			}
			compass_calibration_conclude_xy();
			break;
		}
	}
}
void compass_calibration_z(void)
{
	while(1){//wait for init calibrating
		if(testing){
			mag_z_bias=0;
			maxMagX=0,maxMagY=0,maxMagZ=0;
			minMagX=0,minMagY=0,minMagZ=0;
			while(testing){
				get_compass_bias();
			}
			compass_calibration_conclude_z();
			break;
		}
	}
}
void get_compass_bias(void)
{
	if(sens.mx>maxMagX)//max
		maxMagX=sens.mx;
	if(sens.mx<minMagX)//min
		minMagX=sens.mx;
	if(sens.my>maxMagY)//max
		maxMagY=sens.my;
	if(sens.my<minMagY)//min
		minMagY=sens.my;
	if(sens.mz>maxMagZ)//max
		maxMagZ=sens.mz;
	if(sens.mz<minMagZ)//min
		minMagZ=sens.mz;		
}
void compass_calibration_conclude_xy(void)
{
	short MXgain,MYgain;
	if((maxMagX - minMagX) >= (maxMagY - minMagY)){
		MXgain = 1;
		MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
	}
	else{ //(maxMagY - minMagY) > (maxMagX - minMagX)
		MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
		MYgain = 1;
  	}
	mag_x_bias = -0.5 * MXgain * (maxMagX + minMagX);
	mag_y_bias = -0.5 * MYgain * (maxMagY + minMagY);
}
void compass_calibration_conclude_z(void)
{
	short MZgain;
	if(((maxMagX - minMagX) >= (maxMagY - minMagY)) 
		&& ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
		MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);	 
	else if(((maxMagY - minMagY) > (maxMagX - minMagX)) 
	&& ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
		MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
	else if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) 
	&& ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
		MZgain = 1.0;
	mag_z_bias = -0.5 * MZgain * (maxMagZ + minMagZ);
}
void gyro_calibration(void)
{
	long temp1=0,temp2=0,temp3=0;
	int i;
	gyr_x_bias=0;gyr_y_bias=0;gyr_z_bias=0;
	for(i=0;i<200;i++)
	{
		temp1+=sens.gx;
		temp2+=sens.gy;
		temp3+=sens.gz;
		delay_ms(5);
	}
	gyr_x_bias=-temp1/200;
	gyr_y_bias=-temp2/200;
	gyr_z_bias=-temp3/200;
}
void get_imu_wait(void)
{
	typedef union myun{
		char b[2];
		short c;
	}un;
	un gx_get,gy_get,gz_get;
	un ax_get,ay_get,az_get;
	un mx_get,my_get,mz_get; 
	char IMU_received[12];
	char compass_received[6];
	short temp;
	short gx,gy,gz,ax,ay,az,mx,my,mz;
	int compass_available;
	twi_read_all(IMU_received);
#if MPU_COMPASS
	compass_available = twi_readcompass(compass_received);
#elif HMC_COMPASS
	compass_available = hmc5883_read(compass_received);
#endif
	ax_get.b[1] = IMU_received[0];
	ax_get.b[0] = IMU_received[1];
	ay_get.b[1] = IMU_received[2];
	ay_get.b[0] = IMU_received[3];
	az_get.b[1] = IMU_received[4];
	az_get.b[0] = IMU_received[5];
	gx_get.b[1] = IMU_received[6];
	gx_get.b[0] = IMU_received[7];
	gy_get.b[1] = IMU_received[8];
	gy_get.b[0] = IMU_received[9];
	gz_get.b[1] = IMU_received[10];
	gz_get.b[0] = IMU_received[11];
#if MPU_COMPASS
	mx_get.b[0] = compass_received[0];
	mx_get.b[1] = compass_received[1];
	my_get.b[0] = compass_received[2];
	my_get.b[1] = compass_received[3];
	mz_get.b[0] = compass_received[4];
	mz_get.b[1] = compass_received[5];
#elif HMC_COMPASS
	mx_get.b[1] = compass_received[0];
	mx_get.b[0] = compass_received[1];
	my_get.b[1] = compass_received[2];
	my_get.b[0] = compass_received[3];
	mz_get.b[1] = compass_received[4];
	mz_get.b[0] = compass_received[5];
#endif
	ax = ax_get.c;
	ay = ay_get.c;
	az = az_get.c;
	gx = gx_get.c;
	gy = gy_get.c;
	gz = gz_get.c;
	mx = mx_get.c;
	my = my_get.c;
	mz = mz_get.c;
#if MPU_COMPASS
	temp = my;				//mag sensors have different reference
	my = mx;
	mx = temp;
	mz = -mz;
#elif HMC_COMPASS
	temp = mz;
	mz = -my;	
	my = -mx;
	mx = -temp;
#endif	
	sens.gx = gx + gyr_x_bias;
	sens.gy = gy + gyr_y_bias;
	sens.gz = gz + gyr_z_bias;
	sens.ax = ax + acc_x_bias;
	sens.ay = ay + acc_y_bias;
	sens.az = az + acc_z_bias;
	if(compass_available){
		sens.mx = (mx + mag_x_bias);
		sens.my = (my + mag_y_bias);
		sens.mz = (mz + mag_z_bias);
	}
}
void continue_acc_read(void)
{
	twi_acc_read_start(IMU_received);
}
void continue_gyro_read(void)
{
	twi_gyro_read_start(IMU_received+6);
}
void continue_cps_read(void)
{
	twi_cps_read_start(compass_received);
}
void data_conclude(char switcher)
{
	typedef union myun{
		char b[2];
		short c;
	}un;
	un x_get,y_get,z_get;
	short temp;
	switch (switcher){
	case ACC_SWITCH:
		x_get.b[1] = IMU_received[0];
		x_get.b[0] = IMU_received[1];
		y_get.b[1] = IMU_received[2];
		y_get.b[0] = IMU_received[3];
		z_get.b[1] = IMU_received[4];
		z_get.b[0] = IMU_received[5];
		ax = x_get.c + acc_x_bias;
		ay = y_get.c + acc_y_bias;
		az = z_get.c + acc_z_bias;
		break;
	case GYRO_SWITCH:
		x_get.b[1] = IMU_received[6];
		x_get.b[0] = IMU_received[7];
		y_get.b[1] = IMU_received[8];
		y_get.b[0] = IMU_received[9];
		z_get.b[1] = IMU_received[10];
		z_get.b[0] = IMU_received[11];
		sens.gx = x_get.c + gyr_x_bias;
		sens.gy = y_get.c + gyr_y_bias;
		sens.gz = z_get.c + gyr_z_bias;		
		break;
	case CPS_SWITCH:
		x_get.b[1] = compass_received[0];
		x_get.b[0] = compass_received[1];
		y_get.b[1] = compass_received[2];
		y_get.b[0] = compass_received[3];
		z_get.b[1] = compass_received[4];
		z_get.b[0] = compass_received[5];
		temp = z_get.c;
		z_get.c = -y_get.c;	
		y_get.c = -x_get.c;
		x_get.c = -temp;
		sens.mx = x_get.c + mag_x_bias;
		sens.my = y_get.c + mag_y_bias;
		sens.mz = z_get.c + mag_z_bias;
		break;
	default:
		break;
	}
}
void acc_lowpass(void)
{
	#if ACC_LOWPASS
	sens.ax = IIR_apply(&iir_ax, ax);
	sens.ay = IIR_apply(&iir_ay, ay);
	sens.az = IIR_apply(&iir_az, az);
	#else
	sens.ax = ax;
	sens.ay = ay;
	sens.az = az;
	#endif
}
void imu_IIR_init(void)
{	
	float cutoff_freq = 25.0;
	float smpl_freq = 500.0;
	IIR_set_cutoff_freq(&iir_ax, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_ay, cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_az, cutoff_freq, smpl_freq);
	sens.ax = IIR_reset(&iir_ax, 0);
	sens.ay = IIR_reset(&iir_ay, 0);
	sens.az = IIR_reset(&iir_az, 8192);
}
