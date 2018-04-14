#ifndef PID_H_
#define  PID_H_
#include "global.h"
#ifdef __cplusplus
extern "C"{
#endif
int external_pid(PID *pid, int measured, int setpt, short dt);
int internal_pid(PID *pid, int measured, int setpt, short dt);
#ifdef __cplusplus
}
#endif
void pid_calculate(short dt);
//void reset_variables(void);
void throtle_control(short dt);
#endif
