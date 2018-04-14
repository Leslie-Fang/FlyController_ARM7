#ifndef TWI_FAST_H
#define TWI_FAST_H
typedef struct {
	char *Buf;
	int intAdd[6];
	int devAdd;
	int readCnt;
//	int sizeRemain;
}TWI_Reader ;
void twi_fast_init(void);
int twi_cps_read_start(char *buf);
int twi_gyro_read_start(char *buf);
int twi_acc_read_start(char *buf);
__irq void twi_int_handler(void);
#define ACC_SWITCH 0
#define GYRO_SWITCH 1
#define CPS_SWITCH 2
#define NON_WORKING 3
#define READ_START_NOT_RDY 1
#endif
