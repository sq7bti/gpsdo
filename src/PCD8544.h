#ifndef PCD8544_H_
#define PCD8544_H_

#define LCD5110_SCLK_PIN            BIT5
#define LCD5110_DN_PIN              BIT7

#define LCD5110_RST_PIN             BIT5
#define LCD5110_RESET               P2OUT &= ~LCD5110_RST_PIN
#define LCD5110_DERESET             P2OUT |= LCD5110_RST_PIN

#define LCD5110_BL_PIN              BIT4
#define LCD5110_BL_ON               P2OUT &= ~LCD5110_BL_PIN
#define LCD5110_BL_OFF              P2OUT |= LCD5110_BL_PIN

#define LCD5110_SCE_PIN             BIT0
#define LCD5110_SELECT              P2OUT &= ~LCD5110_SCE_PIN
#define LCD5110_DESELECT            P2OUT |= LCD5110_SCE_PIN

#define LCD5110_DC_PIN              BIT3
#define LCD5110_SET_COMMAND         P2OUT &= ~LCD5110_DC_PIN
#define LCD5110_SET_DATA            P2OUT |= LCD5110_DC_PIN

#define LCD5110_COMMAND             0
#define LCD5110_DATA                1

void setAddr(unsigned char xAddr, unsigned char yAddr);
void writeToLCD(unsigned char dataCommand, unsigned char data);
void writeCharToLCD(char c);
void writeStringToLCD(const char *string);
void initLCD();
void clearLCD();
void clearBank(unsigned char bank);
char dtohex(int i);

#endif /*PCD8544_H_*/
