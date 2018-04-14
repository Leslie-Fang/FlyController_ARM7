#include "inout.h"
#include "global.h"
#include "mypwm.h"
#include "attitude.h"
#include "main.h"
void force2output(int force[4], unsigned short duty[4], unsigned int battery);
void command_init(void)
{
	cmd.pos_z_sp = pos.z_est[0];	
	cmd.pos_x_sp = pos.x_est[0];
	cmd.pos_y_sp = pos.y_est[0];	
}
void get_rc(short dt)
{
	int expctBodyVel[2];
	int expctGlobVel[2];
	int expctAltRate,expctYawRate;
	//mode change
	mode.l_FlightMode = mode.FlightMode;
	if (cmd.rc[4] > 307){//switch upward	
		mode.FlightMode = POS_CTRL;	
	}
	else if(cmd.rc[4] > -307){		//in the middle
		mode.FlightMode = ALT_CTRL;
	}
	else{		//downward
		mode.FlightMode = MANUEL;
	}
	mode.l_CalMode = mode.CalibrationMode;
	mode.CalibrationMode = 0;
	//pitch roll att command for acrob, manuel, altctrl
	if(mode.FlightMode == MANUEL || mode.FlightMode==ACROBATIC || mode.FlightMode == ALT_CTRL){
		cmd.pitch_sp = dead_zone(cmd.rc[1] * MAX_ATT_MANUEL>>10, 0);
		cmd.roll_sp = dead_zone(-cmd.rc[0] * MAX_ATT_MANUEL>>10, 0);
	}
	//xy pos command for posctrl
	else if(mode.FlightMode == POS_CTRL){
		expctBodyVel[0] = dead_zone(cmd.rc[1] * MAX_XY_VEL_MANUEL>>10, 400);
		expctBodyVel[1] = dead_zone(cmd.rc[0] * MAX_XY_VEL_MANUEL>>10, 400);
		body2glob(expctBodyVel, expctGlobVel, 2);
		cmd.pos_x_sp += expctGlobVel[0] * dt / 1000;	
		cmd.pos_y_sp += expctGlobVel[1] * dt / 1000;
		cmd.vel_x_ff = expctGlobVel[0];
		cmd.vel_y_ff = expctGlobVel[1];
	}												
	//yaw command for all
	expctYawRate = dead_zone(cmd.rc[3] * MAX_YAW_RATE_MANEUL>>10, 4260);//deadzone 15 deg
	cmd.yaw_sp += expctYawRate * dt>>10;		
	//throtle command for manuel, acrob
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC){
		cmd.Thrust = dead_zone(constrain((cmd.rc[2] + 1024)/2 * thrCmndRatio, 0, 1400), 50);
	}
	//alt command for altctrl, posctrl
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL){
		expctAltRate = dead_zone(cmd.rc[2] * MAX_ALT_VEL_MANUEL>>10, 200);
		cmd.pos_z_sp += expctAltRate * dt / 1000;
		cmd.vel_z_ff = expctAltRate;
	}
	//tuning
	pos_xPID.Prate = 2.0 + (cmd.rc[5]/1024.0)*1.0;
	pos_xPID.P = 0.22 + (cmd.rc[6]/1024.0)*0.10;
//	pos_xPID.Drate = 0.1 + (cmd.rc[7]/1024.0)*0.1;
	
	pos_yPID.P = pos_xPID.P;
	pos_yPID.Prate = pos_xPID.Prate;	
//	pos_yPID.Irate = pos_xPID.Irate;
//	pos_yPID.Drate = pos_xPID.Drate;	
//	altPID.Prate = 1.99 + (cmd.rc[5]/1024.0)*0.7;
//	altPID.P = 2.26 + (cmd.rc[6]/1024.0)*0.7;
//	altPID.Irate = 0.05 + (cmd.rc[7]/1024.0)*0.05;
	
	pitchPID.Prate = 22.5 + (cmd.rc[5]/1024.0)*8.0;
	pitchPID.P = 2.1 + (cmd.rc[6]/1024.0)*1.0;
//	pitchPID.Drate = 0.03 + (cmd.rc[6]/1024.0)*0.03;
//	pitchPID.Irate = 1.8 + (cmd.rc[7]/1024.0)*1.8;
//	rollPID.Prate = 11.0 + (cmd.rc[5]/1024.0)*9.0;
//	rollPID.Drate = 0.03 + (cmd.rc[7]/1024.0)*0.03;
	rollPID.Prate = pitchPID.Prate;
	rollPID.P = pitchPID.P;
//	rollPID.Irate = pitchPID.Irate;
//	yawPID.Prate = 10.0 + (cmd.rc[5]/1024.0)*8.0;
//	yawPID.P = 1.2 + (cmd.rc[6]/1024.0)*0.9;
}
void put_motors(void)
{
	int motorForce[4] = {0,0,0,0};
	unsigned short motorDuty[4] = {2400,2400,2400,2400};
	short i;
	if(cmd.rc[2] < -850){
		motorForce[0] = 0;
		motorForce[1] = 0;
		motorForce[2] = 0;
		motorForce[3] = 0;
		for(i=0;i<4;i++){
			motorDuty[i]=2400;
		}
	}
	else{
	#if PLUS
/*			A								inv(A)
			[  1,  1, 1,  1]				[ 1/4, -1/(2*d),        0,  1/(4*c)]
			[ -d,  0, d,  0]				[ 1/4,        0, -1/(2*d), -1/(4*c)]
			[  0, -d, 0,  d]				[ 1/4,  1/(2*d),        0,  1/(4*c)]
			[  c, -c, c, -c]				[ 1/4,        0,  1/(2*d), -1/(4*c)]*/
		motorForce[0] = (output.thrustForce>>2) - output.pitchMmt / (ROTOR_DIST<<1) + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[1] = (output.thrustForce>>2) - output.rollMmt / (ROTOR_DIST<<1) - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[2] = (output.thrustForce>>2) + output.pitchMmt / (ROTOR_DIST<<1) + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[3] = (output.thrustForce>>2) + output.rollMmt / (ROTOR_DIST<<1) - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
	#elif CROSS
/*			A													inv(A)
			[        1,        1,        1,       1]			[ 1/4, -sqrt2/(4*d),  sqrt2/(4*d),  1/(4*c)]
			[ -d/sqrt2, -d/sqrt2,  d/sqrt2, d/sqrt2]			[ 1/4, -sqrt2/(4*d), -sqrt2/(4*d), -1/(4*c)]
			[  d/sqrt2, -d/sqrt2, -d/sqrt2, d/sqrt2]			[ 1/4,  sqrt2/(4*d), -sqrt2/(4*d),  1/(4*c)]
			[        c,       -c,        c,      -c]			[ 1/4,  sqrt2/(4*d),  sqrt2/(4*d), -1/(4*c)]
		d=D/2
		torq=c*f*/
		motorForce[0] = (output.thrustForce>>2) + (-output.pitchMmt + output.rollMmt) / D2_SQRT2 + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[1] = (output.thrustForce>>2) + (-output.pitchMmt - output.rollMmt) / D2_SQRT2 - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[2] = (output.thrustForce>>2) + (output.pitchMmt - output.rollMmt) / D2_SQRT2 + output.yawMmt / (FORCE_TORQUE_RATIO<<2);
		motorForce[3] = (output.thrustForce>>2) + (output.pitchMmt + output.rollMmt) / D2_SQRT2 - output.yawMmt / (FORCE_TORQUE_RATIO<<2);
	#endif		
		force2output(motorForce, motorDuty, adc.battery);
	}	
	//channel-PWMlinesign order- 1324
	pwm_set_duty_cycle(0, constrain(motorDuty[0],2400,4790));
	pwm_set_duty_cycle(1, constrain(motorDuty[1],2400,4790));
	pwm_set_duty_cycle(2, constrain(motorDuty[2],2400,4790));
	pwm_set_duty_cycle(3, constrain(motorDuty[3],2400,4790));
}
void force2output(int force[4], unsigned short duty[4], unsigned int battery)
//mNewton to 0~2400
{
	#if F450
		#define C 310
	#elif XINSONG
		#define C 410
	#elif F330
		#define C 300
	#elif F240
		#define C 200	
	#endif
	int k,i;
	if(battery == 0){
		k = 110;
	} else{
		//full4.20V(4.05V) k=100, low3.80V(3.65V) k=120
		k = constrain(303-(int)battery/20,95,125);
	}
	for(i=0;i<4;i++){
		duty[i] = force[i] * k / C + 2650;
	}
}
//insert a into [b,c]
//#define constrain(a,b,c) ((a)<(b)?(b):(a)>(c)?c:a)
int constrain(int a, int b, int c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
//dead zone +-b
//#define dead_zone(a,b) ((a)>(b)?(a):(a)<(-b)?(a):0)
int dead_zone(int a, int b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}

//#define interpol(x,x1,y1,x2,y2) (y1+(y2-y1)/(x2-x1)*(x-x1))
//#define myabs(a) ((a)>=0?(a):(-a))

float constrain_f(float a, float b, float c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
int minimum(int a, int b){
	return (a>b?b:a);
}
int maximum(int a, int b){
	return (a>b?a:b);
}
