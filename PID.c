#include "PID.h"
#include "global.h"
#include "attitude.h"
#if EULER_CTRL_PID
#if F330
PID pitchPID={0,0,0,0,
			1.45,
			14.9,0.0,0.006};//I0.0082
PID rollPID={0,0,0,0,
			1.45,
			17.2,0.0,0.006};
PID yawPID={0,0,0,0,
			3.0,
			34.4,0,0};

PID altPID={0,0,0,0,
			0.408,
			0.25,0.0002,0.00017};//0.0068,0.0218};

PID pos_xPID={0,0,0,0,
			0.65,
			1.14,0.0,0.0};//0.0076};
PID pos_yPID={0,0,0,0,
			0.48,
			0.9,0.0,0.00385};//0.0076};
			
#elif F240
PID pitchPID={0,0,0,0,
			1.5,
			0.16,0.3,0.0002};
PID rollPID={0,0,0,0,
			1.5,
			0.16,0.03,0.0002};
PID yawPID={0,0,0,0,
			2.0,
			0.4,0,0};
incrPID altPID={0,0,0,0,
				0.0,
				2.17,0.0,0,
				0};
PID pos_xPID={0,0,0,0,
			0,
			0,0,0};
PID pos_yPID={0,0,0,0,
			0,
			0,0,0};
#endif
#endif

int external_pid(PID *pid, int measured, int setpt, short dt)
{
 	 int retValue;								
 	 pid->Err = setpt - measured;
 	 retValue = pid->P * pid->Err;
	 return retValue;
}
int internal_pid(PID *pid, int measured, int setpt, short dt)
{
   	int rateOuput;								
  	pid->RateErr = setpt - measured;
  	rateOuput = pid->Prate * pid->RateErr + pid->Drate * (pid->RateErr - pid->l_RateErr) *1000/ dt + pid->Irate * pid->int_RateErr;
  	pid->l_RateErr = pid->RateErr;
	pid->int_RateErr += pid->RateErr * dt/1000;
  	return rateOuput;
}
void pid_calculate(short dt)
{
	int PitchRate_sp, RollRate_sp, YawRate_sp;
	int globVel_sp[2],globAcc_sp[2],bodyAcc_sp[2];
	if(mode.FlightMode == POS_CTRL || mode.FlightMode == XY_CTRL){
//get attitude setpoint from position setpoint
		globVel_sp[0] = constrain(external_pid(&pos_xPID, pos.x_est[0], cmd.pos_x_sp, dt), -MAX_XY_VEL, MAX_XY_VEL);
		globVel_sp[1] = constrain(external_pid(&pos_yPID, pos.y_est[0], cmd.pos_y_sp, dt), -MAX_XY_VEL, MAX_XY_VEL);
		globAcc_sp[0] = internal_pid(&pos_xPID, pos.x_est[1], globVel_sp[0],dt);
		globAcc_sp[1] = internal_pid(&pos_yPID, pos.y_est[1], globVel_sp[1],dt);
		glob2body(bodyAcc_sp, globAcc_sp, 2);
		cmd.pitch_sp = constrain((bodyAcc_sp[0]<<14) / GRAVITY , -MAX_ATT_SP,MAX_ATT_SP);
		cmd.roll_sp = constrain(-(bodyAcc_sp[1]<<14) / GRAVITY, -MAX_ATT_SP,MAX_ATT_SP);
	}
	else if(mode.FlightMode == POS_RATE_CTRL){
		globVel_sp[0] = cmd.pos_x_sp;
		globVel_sp[1] = cmd.pos_y_sp;
		globAcc_sp[0] = internal_pid(&pos_xPID, pos.x_est[1], globVel_sp[0],dt);
		globAcc_sp[1] = internal_pid(&pos_yPID, pos.y_est[1], globVel_sp[1],dt);
		glob2body(bodyAcc_sp, globAcc_sp, 2);
		cmd.pitch_sp = constrain((bodyAcc_sp[0]<<14) / GRAVITY , -MAX_ATT_SP,MAX_ATT_SP);
		cmd.roll_sp = constrain(-(bodyAcc_sp[1]<<14) / GRAVITY, -MAX_ATT_SP,MAX_ATT_SP);
	}
//external pid for attitude
	PitchRate_sp = external_pid(&pitchPID, att.pitch, cmd.pitch_sp, dt);
	RollRate_sp = external_pid(&rollPID, att.roll, cmd.roll_sp, dt);
	YawRate_sp = constrain(external_pid(&yawPID, att.yaw, cmd.yaw_sp, dt), -MAX_YAW_RATE, MAX_YAW_RATE);
//internal pid for attitude
	if(mode.FlightMode == ACROBATIC){
	//command directly into attitude internal loop
		PitchRate_sp = cmd.pitch_sp;
		RollRate_sp = cmd.roll_sp;
		YawRate_sp = cmd.yaw_sp;
	}
	//external result into attitude internal loop

	//now doing fixed point unification, the result is 16384 times larger
	//and in the end >>4, it is 1024 times larger
	output.pitch = internal_pid(&pitchPID, att.pitchspeed, PitchRate_sp, dt)>>4; 
	output.roll = internal_pid(&rollPID, att.rollspeed, RollRate_sp, dt)>>4;
	output.yaw = internal_pid(&yawPID, att.yawspeed, YawRate_sp, dt)>>4;
//thrust control
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC || mode.FlightMode == XY_CTRL){
	//pass command directly into output				
		output.thrust = cmd.Thrust;	
		output.ref_thrust = output.thrust;
	}
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL||mode.FlightMode == POS_RATE_CTRL){
	//expctAlt -> AltRate -> thrust, compliment for the inclination
		cmd.altRate_sp = constrain(external_pid(&altPID, pos.z_est[0], cmd.alt_sp, dt), -MAX_ALT_VEL, MAX_ALT_VEL);
		output.thrust = constrain(output.ref_thrust + 
						(internal_pid(&altPID, pos.z_est[1], cmd.altRate_sp, dt)<<10)/1000, 
						MIN_THRUST, MAX_THRUST);
		//originally, the result of internal pid is in m, ref thrust is 0~1
		//the result of internal pid is in mm, ref_thrust is 0~1024
		
		if(0)//AttComp
			output.thrust = (output.thrust<<14) / att.R[2][2];
	}
	else if(mode.FlightMode==ALT_RATE_CTRL){
		output.thrust = constrain(output.ref_thrust + 
						(internal_pid(&altPID, pos.z_est[1], cmd.altRate_sp, dt)<<10)/1000,
						 MIN_THRUST, MAX_THRUST);
	}
}
/*void reset_variables(void)
{
	altPID.int_RateErr = 0;
	pitchPID.int_RateErr = 0;
	rollPID.int_RateErr = 0;
	yawPID.int_RateErr = 0;
	pos_xPID.int_RateErr = 0;
	pos_yPID.int_RateErr = 0;
	cmd.yaw_sp = att.yaw;
	cmd.alt_sp = pos.z_est[0];
}
*/
void throtle_control(short dt)
{
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC || mode.FlightMode == XY_CTRL){
	//pass command directly into output				
		output.thrust = cmd.Thrust;	
		output.ref_thrust = output.thrust;
	}
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL||mode.FlightMode == POS_RATE_CTRL){
	//expctAlt -> AltRate -> thrust, compliment for the inclination
		cmd.altRate_sp = constrain(external_pid(&altPID, pos.z_est[0], cmd.alt_sp, dt), -MAX_ALT_VEL, MAX_ALT_VEL);
		output.thrust = constrain(output.ref_thrust + 
						((int)internal_pid(&altPID, pos.z_est[1], cmd.altRate_sp, dt)<<10)/1000, 
						MIN_THRUST, MAX_THRUST);
		//originally, the result of internal pid is in m, ref thrust is 0~1
		//the result of internal pid is in mm, ref_thrust is 0~1024
		
		if(0)//AttComp
			output.thrust = (output.thrust<<14) / att.R[2][2];
	}
}
