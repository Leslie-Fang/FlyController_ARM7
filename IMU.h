#ifndef IMU_H
#define IMU_H
void get_compass_bias(void);
void compass_calibration_conclude_xy(void);
void compass_calibration_conclude_z(void);
extern void compass_calibration_xy(void);
extern void compass_calibration_z(void);
extern void gyro_calibration(void);
extern void get_imu_wait(void);
void continue_gyro_read(void);
void continue_acc_read(void);
void data_conclude(char switcher);
void continue_cps_read(void);
void acc_lowpass(void);
extern void imu_IIR_init(void);
#endif
