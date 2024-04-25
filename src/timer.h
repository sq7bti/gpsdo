#ifndef TIMER_H
#define TIMER_H

#include "common.h"

void initTIMER(void);

uint8_t getticks();
uint8_t getclicks();
uint32_t getMillis();
uint32_t getSeconds();
uint16_t getOCXO();
uint32_t getCAP();

#endif /* TIMER_H */
