#ifndef PCD8544_H_
#define PCD8544_H_

#define LCD5110_SCLK_PIN            BIT5
#define LCD5110_DN_PIN              BIT7

#define LCD5110_SCE_PIN             BIT7
#define LCD5110_SELECT              P2OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT            P2OUT |= LCD5110_SCE_PIN

#define LCD5110_DC_PIN              BIT3
#define LCD5110_SET_COMMAND         P2OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA            P2OUT |= LCD5110_DC_PIN

#define LCD5110_COMMAND             0
#define LCD5110_DATA                1

#include  "common.h"

void setAddr(unsigned char xAddr, unsigned char yAddr);
void setInverse(bool s);
void writeToLCD(unsigned char dataCommand, unsigned char data);
void writeCharToLCD(char c);
void writeStringToLCD(const char *string);
void writeDecToLCD(uint32_t i);
void writeIntToLCD(int32_t i);
void writeMHzToLCD(uint32_t i);
void writeQ88ToLCD(uint16_t i);
void writeQ4CToLCD(uint16_t i);
void writeByteToLCD(uint8_t i);
void writeWordToLCD(uint16_t i);
void initLCD();
void clearLCD();
void clearBank(unsigned char bank);
char i2h(uint8_t i);
void pixel(uint8_t x, uint8_t y);
void bargraph(uint8_t l, uint16_t val);
void phase_difference(uint8_t l, uint16_t val, uint16_t marker);

#endif /*PCD8544_H_*/
