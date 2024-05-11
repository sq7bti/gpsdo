#ifndef TIMER_H
#define TIMER_H

#include "common.h"

#define TA0CCR1_DEF 0x73FD

#define TRIGGER_PID 7
#define TRIGGER_SLW 4
#define TRIGGER_ADC 3
#define TRIGGER_SEC 2
#define TRIGGER_LOG 1
#define TRIGGER_LCD 0
#define TRIGGER_ANY -1

void initTIMER(void);

uint8_t getclicks();
uint32_t getMillis();
uint32_t getSeconds();
uint32_t getOCXO();
uint32_t getCAP();
void setPWM(uint16_t);
uint16_t getPWM();
bool getTrigFlag(int8_t id);
void setTrigFlag(uint8_t id);
void clearTrigFlag(uint8_t id);
int16_t getPhaseDiff();
#endif /* TIMER_H */
