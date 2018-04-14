#ifndef GLOBAL_H
#define GLOBAL_H

#define BOARD_V5 1
#define BOARD_V4 0
#define F450 1
#define XINSONG 0
#define F330 0
#define F240 0
#if F240
	#define PLUS 1
#else
	#define CROSS 1
#endif
#define INDOOR 0
#define OUTDOOR 1

#define M8N_GPS 1
#define LEA6H_GPS 0
#define HMC_COMPASS 1
#define MPU_COMPASS 0

#define ON_FLIGHT 1
#define OFF_FLIGHT 0
#define WAIT_GPS 1
#define OLD_ATT 1
#define NEW_ATT 0

#define ORIGINAL_FREQ 1
#define DOUBLED_FREQ 0

#if OUTDOOR
struct _gps{
	long long lat;//10^7deg
	long long lon;
	int alt;
	int vel;
	float azm; //azimuth, rad
	short sat;
	short status;
	char gpsflag;
	int x;
	int y;
	int z;
	int vx;
	int vy;
	int vz;	
	#define GPS_PERIOD 200
	#define GPS_DELAY 400
};
extern struct _gps gps;
#elif INDOOR
struct _vicon{
	int x;//mm
	int y;
	int z;
	int vx;//mm/s
	int vy;
	int vz;
	char xbeeflag;
	#define VICON_PERIOD 100//ms
	#define VICON_DELAY 120
};
extern struct _vicon vicon;
#endif
struct _smpl{
#define PIT_FREQ 5000
#define ATT_PERIOD 2
#define POS_CTRL_PERIOD 4
#define RADIO_PERIOD 20
#define BARO_PERIOD 4
//	long Count500Hz;
	char Flag500Hz;
//	long UARTreceiveCount;
//	short halfT;
	short baroStep;
	char tx_finished;
	unsigned char vel_filtered;
};
extern struct _smpl smpl;

struct _att {
	int pitch;
	int roll;
	int yaw;
	int pitchspeed;
	int rollspeed;
	int yawspeed;
	int R[3][3];//glob=R*body//1 ~ 2^14 ~ 16384
};
extern struct _att att;

struct _sens {
	short gx;
	short gy;
	short gz;
	short ax;
	short ay;
	short az;
	short mx;
	short my;
	short mz;	 
	unsigned char mag_updated;
};
extern struct _sens sens;

struct _mode {
	char FlightMode;
	char CalibrationMode;
	char l_FlightMode;
	char l_CalMode;
	#define MANUEL 1
	#define ACROBATIC 0
	#define ALT_CTRL 2	
	#define POS_CTRL 4
	#define MOTOR_CUT 9
};
extern struct _mode mode;

struct _baro {
	int temp;
	int pressure;
	int refPressure;	 
	int alt;
	char temp_pres_switch;
	#define PRES_SWITCH 0
	#define TEMP_SWITCH 1
	#define BARO_STEPS 4
};
extern struct _baro baro;

struct _pos {	 //unit mm
	float x_est[2];
	float y_est[2];
	float z_est[2];
	int Acc_x;
	int Acc_y;
	int Acc_z;
	int sonarPos;	
};
extern struct _pos pos;

struct _cmd {
	short rc[9];//-1024 ~ +1024
	
	int pitch_sp;
	int roll_sp;
	int yaw_sp;	
			
	int pos_x_sp;
	int pos_y_sp;
	int pos_z_sp;
	
	int vel_x_sp;
	int vel_y_sp;
	int vel_z_sp;
	
	int vel_x_ff;
	int vel_y_ff;
	int vel_z_ff;
	
	int Thrust;//0~1024
	char SonarEnable;
	char data2send;
	#define SonarOFF 0
	#define SonarON 1
	#define sendNON 0
	#define sendSENS 1
	#define sendGPS 2
	#define sendATT 3
	#define sendPOS 4
	#define sendPID 5
	#define sendCMD 6
	#define sendOUT 7
	#define KILL 11
};
extern struct _cmd cmd;

struct _output {
	int thrustForce;//U1=F1+F2+F3+F4, 1 mNewton
	int pitchMmt;//U1=(F3-F1)L
	int rollMmt;//U2=(F2-F4)L
	int yawMmt;//U4=M2+M4-M1-M3, 1 mNewton*mm
//	int ref_thrust;
};
extern struct _output output;

struct _adc {
	unsigned int battery;//in mV
};
extern struct _adc adc;
struct _myusb {
	#define PLUG_IN 1
	#define PLUG_OUT 2
	#define PLUG_CLEAR 0
	char connect_flag;
	char connected;
	short data_got[3];
};
extern struct _myusb myusb;

extern short data2[9];
typedef struct _PID {
	int Err;
	int RateErr;
	int l_RateErr;
	int int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
}PID;
extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;
extern PID altPID;
extern PID pos_xPID;
extern PID pos_yPID;
#define RAD2DEG 57.3
#define DEG2RAD 0.0174
#define PI 3.1416
#define EARTH_RADIAS 6371000//m
#define GRAVITY 9810
#define HALF_SQRT_2 0.7071
#if F450
	#define thrCmndRatio 1
#elif XINSONG
	#define thrCmndRatio 1
#elif F330
	#define thrCmndRatio 1
#elif F240
	#define thrCmndRatio 13/5
#endif
//#define MAX_ATT_SP 5718//20deg,0.349rad
#define MAX_ATT_MANUEL 14437//11437 40deg,0.698rad
#define MAX_ALT_VEL 1000//in loop
#define MAX_ALT_VEL_MANUEL 500//manuel
#define MAX_XY_VEL 80000
#define MAX_XY_VEL_MANUEL 1500
#define MAX_YAW_RATE 14303//50.0
#define MAX_YAW_RATE_MANEUL 14303

#define BAT_WARNING 3500
#if F450
	#define VEHICLE_MASS 1300//g  now take 1300 actually 1300g,should be 1450
	#define ROTOR_DIST 450//mm
	#define D2_SQRT2 636//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15//torq=c*f
#elif XINSONG
	#define VEHICLE_MASS 1200//g
	#define ROTOR_DIST 450//mm
	#define D2_SQRT2 636//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15//torq=c*f
#elif F330
	#define VEHICLE_MASS 1100//actually 920g
	#define ROTOR_DIST 330//mm
	#define D2_SQRT2 467//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15
#elif F240
	#define VEHICLE_MASS 448
	#define ROTOR_DIST 240//mm
	#define D2_SQRT2 339//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15
#endif
extern int constrain(int a, int b, int c);
extern int dead_zone(int a, int b);
extern float constrain_f(float a, float b, float c);
extern int minimum(int a, int b);
extern int maximum(int a, int b);
extern struct _AT91S_CDC 	pCDC;

#endif
