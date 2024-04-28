#ifndef TIMER_H
#define TIMER_H

#include "common.h"

#define TA0CCR1_DEF 0x2770

void initTIMER(void);

uint8_t getclicks();
uint32_t getMillis();
uint32_t getSeconds();
uint16_t getOCXO();
uint32_t getCAP();

#endif /* TIMER_H */
