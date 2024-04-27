#ifndef ADC_H_
#define ADC_H_
#include "common.h"

uint16_t getOCXOTemperature();
uint16_t getIntTemperature();
uint16_t getADC();
uint8_t getticks();
void initADC();

#endif /* ADC_H_ */
