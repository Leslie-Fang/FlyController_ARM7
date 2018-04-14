#include "PosEst.h"
#include "global.h"
#include "math.h"
#include "attitude.h"
#include "string.h"
#define acc_bais_corr_weight 0.00005
#define sonar_weight 1.0
#if OUTDOOR
#define gps_xy_weight 0.8
#define gps_vxy_weight 1.2
#define gps_z_weight 0.0
#define baro_weight 0.75
#elif INDOOR
#define vicon_xy_weight 4.0
#define vicon_z_weight 4.0
#define vicon_vxy_weight 1.0
#define vicon_vz_weight 1.0
#define baro_weight 0.0
#endif
short buf_ptr = 0;
#define EST_BUF_SIZE 40//80ms
float est_buf[EST_BUF_SIZE][3][2];
int R_buf[EST_BUF_SIZE][3][3];
#define BUF_INTERVAL 20//ms, 50Hz
float acc_body_bias[3] = {0,0,0};
#if OUTDOOR
int home_lat, home_lon, home_gps_alt;
int cos_lat_100;
int l_x=0,l_y=0,l_z=0;
void gps_pos_init(void)
{
	unsigned short i;
	home_lat = gps.lat;
	home_lon = gps.lon;
	home_gps_alt = gps.alt;
	pos.x_est[0] = 0;
	pos.y_est[0] = 0;
	pos.x_est[1] = 0;
	pos.y_est[1] = 0;
	for(i=0;i<EST_BUF_SIZE;i++){
		est_buf[i][0][0] = gps.x;
		est_buf[i][0][1] = 0;
		est_buf[i][1][0] = gps.y;
		est_buf[i][1][1] = 0;
		est_buf[i][2][0] = baro.alt;
		est_buf[i][2][1] = 0;
	}
//	l_x=0;
//	l_y=0;
	cos_lat_100 = cos(home_lat / 5.72957e+8f)*100.0f;
}
void gps2xyz(short dt)
{
	int equator_distance = (gps.lon - home_lon) * EARTH_RADIAS /572957;
	//(gps.lon - home_lon)(unit)/57.3(deg/rad)/1000000(unit/deg) * EARTH_RADIAS(m) * 1000(mm/m);
	gps.y = (gps.lat - home_lat) * EARTH_RADIAS /572957;
	gps.x = equator_distance * cos_lat_100 / 100;
	gps.z = (gps.alt - home_gps_alt)*1000;
	if(gps.azm != 0.0f){	
		gps.vx=gps.vel * sin(gps.azm);//north is 0, east is 90
		gps.vy=gps.vel * cos(gps.azm);				
	}
	else{
		gps.vx = (gps.x - l_x) *1000/ dt;
		gps.vy = (gps.y - l_y) *1000/ dt;
	}
	l_x=gps.x;
	l_y=gps.y;
}
void gps_pos_corr(short dt)
{
	unsigned char i;
	float c;
	float glob_acc_bias_corr_x = 0.0f, glob_acc_bias_corr_y = 0.0f;
	int corr_x, corr_y;//, corr_z;
	short corr_vx, corr_vy;//, corr_vz;
	short est_i;
	static char start = 1;
	if(start == 1){
		l_x = gps.x;
		l_y = gps.y;	
		start = 0;
	}	
	gps2xyz(dt);
	est_i = buf_ptr - 1 - minimum(EST_BUF_SIZE - 1,GPS_DELAY / BUF_INTERVAL);
	if (est_i < 0) {
		est_i += EST_BUF_SIZE;
	}
	corr_x = gps.x - est_buf[est_i][0][0];
	corr_y = gps.y - est_buf[est_i][1][0];
	inertial_filter_correct(corr_x, dt, pos.x_est, 0, gps_xy_weight);
	inertial_filter_correct(corr_y, dt, pos.y_est, 0, gps_xy_weight);
	
	corr_vx = gps.vx - est_buf[est_i][0][1];
	corr_vy = gps.vy - est_buf[est_i][1][1];
	inertial_filter_correct(corr_vx, dt, pos.x_est, 1, gps_vxy_weight);
	inertial_filter_correct(corr_vy, dt, pos.y_est, 1, gps_vxy_weight);
	smpl.vel_filtered = 1;
	glob_acc_bias_corr_x = -corr_x * gps_xy_weight * gps_xy_weight - corr_vx * gps_vxy_weight;
	glob_acc_bias_corr_y = -corr_y * gps_xy_weight * gps_xy_weight - corr_vy * gps_vxy_weight;
	for(i=0;i<3;i++){
		c = glob_acc_bias_corr_x * R_buf[est_i][0][i];
		c += glob_acc_bias_corr_y * R_buf[est_i][1][i];
		c /= 16384.0;
		acc_body_bias[i] += c * dt * acc_bais_corr_weight;
	}
}
#elif INDOOR
void vicon_pos_init(void)
{
	unsigned int i;
	for(i=0;i<EST_BUF_SIZE;i++){
		est_buf[i][0][0] = vicon.x;
		est_buf[i][0][1] = 0;
		est_buf[i][1][0] = vicon.y;
		est_buf[i][1][1] = 0;
		est_buf[i][2][0] = vicon.z;
		est_buf[i][2][1] = 0;
	}
	pos.x_est[0] = vicon.x;
	pos.y_est[0] = vicon.y;
	pos.z_est[0] = vicon.z;
	pos.x_est[1] = 0;
	pos.y_est[1] = 0;
	pos.z_est[1] = 0;
}
void vicon_pos_corr(short dt)
{
	unsigned char i,j;
	float c;
	float glob_acc_bias_corr[3] = {0.0f, 0.0f, 0.0f};
	short vicon_corr[3][2];
	static short l_x=0,l_y=0,l_z=0;
	static char start = 1;
	short est_i;
	if(start == 1){
		l_x = vicon.x;
		l_y = vicon.y;	
		l_z = vicon.z;
		start = 0;
	}
	est_i = buf_ptr - 1 - VICON_DELAY / BUF_INTERVAL;
	if (est_i < 0) {
		est_i += EST_BUF_SIZE;
	}
	vicon_corr[0][0] = vicon.x - est_buf[est_i][0][0];
	vicon_corr[1][0] = vicon.y - est_buf[est_i][1][0];
	vicon_corr[2][0] = vicon.z - est_buf[est_i][2][0];
	inertial_filter_correct(vicon_corr[0][0], dt, pos.x_est, 0, vicon_xy_weight);
	inertial_filter_correct(vicon_corr[1][0], dt, pos.y_est, 0, vicon_xy_weight);
	inertial_filter_correct(vicon_corr[2][0], dt, pos.z_est, 0, vicon_z_weight);
	
	vicon.vx = (vicon.x - l_x) *1000/ dt;
	vicon.vy = (vicon.y - l_y) *1000/ dt;
	vicon.vz = (vicon.z - l_z) *1000/ dt;
	l_x=vicon.x;
	l_y=vicon.y;
	l_z=vicon.z;			
	vicon_corr[0][1] = vicon.vx - est_buf[est_i][0][1];
	vicon_corr[1][1] = vicon.vy - est_buf[est_i][1][1];
	vicon_corr[2][1] = vicon.vz - est_buf[est_i][2][1];
	inertial_filter_correct(vicon_corr[0][1], dt, pos.x_est, 1, vicon_vxy_weight);
	inertial_filter_correct(vicon_corr[1][1], dt, pos.y_est, 1, vicon_vxy_weight);
	inertial_filter_correct(vicon_corr[2][1], dt, pos.z_est, 1, vicon_vz_weight);

	glob_acc_bias_corr[0] -= vicon_corr[0][0] * vicon_xy_weight * vicon_xy_weight;
	glob_acc_bias_corr[0] -= vicon_corr[0][1] * vicon_vxy_weight;
	glob_acc_bias_corr[1] -= vicon_corr[1][0] * vicon_xy_weight * vicon_xy_weight;
	glob_acc_bias_corr[1] -= vicon_corr[1][1] * vicon_vxy_weight;
	glob_acc_bias_corr[2] -= vicon_corr[2][0] * vicon_z_weight * vicon_z_weight;
	glob_acc_bias_corr[2] -= vicon_corr[2][1] * vicon_vz_weight;
	//because baro o point is different from vicon o point, 
	//when vicon_z is contributing to acc_bias_corr
	//error occurs
	
	for(i=0;i<3;i++){
		c = 0.0f;
		for(j=0;j<3;j++){
			c += R_buf[est_i][j][i] * glob_acc_bias_corr[j];
		}
		acc_body_bias[i] += c * dt * acc_bais_corr_weight / 16384.0f;	
	}
}
#endif
void baro_pos_corr(short dt)//this has same freq as baro update
{
	unsigned char i;
	float c;
	int corr_z_baro =  baro.alt - pos.z_est[0];
	float glob_acc_bais_z = -corr_z_baro * baro_weight * baro_weight;
	inertial_filter_correct(corr_z_baro, dt, pos.z_est, 0, baro_weight);		
	for(i=0;i<3;i++){
		c = glob_acc_bais_z * att.R[2][i] / 16384.0;
		acc_body_bias[i] += c * dt * acc_bais_corr_weight;
	}	
}
void sonar_pos_corr(short dt)//this has same freq as sonar update
{
	int corr_z_sonar;
	if(pos.sonarPos < 200){//ground effect avoid baro
		corr_z_sonar =  pos.sonarPos - pos.z_est[0];
	}
	else{
		corr_z_sonar = 0;
	}	
	inertial_filter_correct(corr_z_sonar, dt, pos.z_est, 0, sonar_weight);
}
void corr_geo_acc(int avgAcc[3])
{
	int i;
	for(i=0;i<3;i++)
		acc_body_bias[i] = avgAcc[i];
}
void get_geo_acc(int globAcc[3])
{
	int bdyAcc[3]={0,0,0};
	short i;
	//加速度计量程必须能容得下振动的峰值，否则算出加速度一直低于0
	bdyAcc[0] = (int)sens.ax * GRAVITY / 8192;
	bdyAcc[1] = (int)sens.ay * GRAVITY / 8192;
	bdyAcc[2] = (int)sens.az * GRAVITY / 8192;
	for(i=0;i<3;i++)
		bdyAcc[i] -= acc_body_bias[i];
//	for(i=0;i<3;i++)
	//	bdyAcc[i] += accStaticCorr[i];
	body2glob(bdyAcc, globAcc, 3);
	globAcc[2] -= GRAVITY;
}
void pos_predict(short dt,unsigned int record_count)
{
	int globAcc[3]={0,0,0};
	get_geo_acc(globAcc);
	pos.Acc_x = globAcc[0];
	pos.Acc_y = globAcc[1];
	pos.Acc_z = globAcc[2];
	inertial_filter_predict(dt, pos.x_est, pos.Acc_x);
	inertial_filter_predict(dt, pos.y_est, pos.Acc_y);
	inertial_filter_predict(dt, pos.z_est, pos.Acc_z);
	if(record_count == 1){
	//50Hz recording
	//totally 0.24s, 12 pages of buffer
		est_buf[buf_ptr][0][0] = pos.x_est[0];
		est_buf[buf_ptr][0][1] = pos.x_est[1];
		est_buf[buf_ptr][1][0] = pos.y_est[0];
		est_buf[buf_ptr][1][1] = pos.y_est[1];
		est_buf[buf_ptr][2][0] = pos.z_est[0];
		est_buf[buf_ptr][2][1] = pos.z_est[1];
		memcpy(R_buf[buf_ptr], att.R, sizeof(att.R));
		buf_ptr++;
		if (buf_ptr >= EST_BUF_SIZE) {
			buf_ptr = 0;
		}
	}
}
void inertial_filter_predict(short dt, float x[2], int acc)//dt in ms, x in mm and mm/s, acc in mm/s2
{	
	x[0] += (x[1] * dt  + (float)acc * dt  * dt / 2000)/1000;
	x[1] += (float)acc * dt / 1000;	
}
void inertial_filter_correct(int e, short dt, float x[2], char i, float w)//e in mm
{
	float ewdt = e * w * dt / 1000;
	x[i] += ewdt;
	if (i == 0){
		x[1] += w * ewdt;
	}
}
