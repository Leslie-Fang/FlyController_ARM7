#ifndef LED_H
#define LED_H
#include "at91sam7s256.h"
#define ON 1
#define OFF 0
void led_init(void);
void led_ctrl(unsigned int channel,unsigned int control);
void beep(unsigned int control);
#endif
