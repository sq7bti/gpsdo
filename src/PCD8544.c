#include <msp430g2553.h>
#include "PCD8544.h"
#include "printf.h"

#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01

#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5

// H = 0
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80
#define PCD8544_HPIXELS 84
#define PCD8544_VBANKS 6
#define PCD8544_MAXBYTES 504 // PCD8544_HPIXELS * PCD8544_VBANKS

// H = 1
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

#define LCD_MAX_X     84
#define LCD_MAX_Y     48

static const char font[][5] = { // basic font
{0x00, 0x00, 0x00, 0x00, 0x00} // 20
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f Deg Symbol
};

static uint8_t prev_bar_val = 255, prev_pd_val = 255, prev_pd_valf = 255;
static uint8_t inverse = 0x00;

// LCD functions implementation

void setAddr(unsigned char xAddr, unsigned char yAddr) {
  writeToLCD(LCD5110_COMMAND, PCD8544_SETXADDR | xAddr);
  writeToLCD(LCD5110_COMMAND, PCD8544_SETYADDR | yAddr);
}

void writeToLCD(unsigned char dataCommand, unsigned char data) {
  LCD5110_SELECT;

  if(dataCommand) {
      LCD5110_SET_DATA;
  } else {
      LCD5110_SET_COMMAND;
  }

  UCB0TXBUF = data;
  while(!(IFG2 & UCB0TXIFG));

  LCD5110_DESELECT;
}

void initLCD() {
  int i;
  LCD5110_SET_COMMAND;
  for(i = 16; i > 0; --i)
    __delay_cycles(100000);

  writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
  writeToLCD(LCD5110_COMMAND, PCD8544_SETVOP | 0x3F);
  writeToLCD(LCD5110_COMMAND, PCD8544_SETTEMP | 0x02);
  writeToLCD(LCD5110_COMMAND, PCD8544_SETBIAS | 0x03);
  writeToLCD(LCD5110_COMMAND, PCD8544_FUNCTIONSET);
  writeToLCD(LCD5110_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

void setInverse(bool s) {
  inverse = s?0xFF:0x00;
};

void writeCharToLCD(char c) {
  unsigned char i;
  for(i = 0; i < 5; i++) {
      writeToLCD(LCD5110_DATA, inverse^font[c - 0x20][i]);
  }
  writeToLCD(LCD5110_DATA, inverse);
}

void writeDecToLCD(uint32_t i) {
  if(i < 10)
    writeCharToLCD(i2h(i));
  else {
    writeDecToLCD(i/10);
    writeCharToLCD(i2h(i%10));
  }
}

void writeIntToLCD(int32_t i) {
  if(i != 0) {
    writeCharToLCD(' ');
  } else {
    if(i > 0) {
      writeCharToLCD('+');
    } else {
      writeCharToLCD('-');
      i = -i;
    }
  }
  writeDecToLCD(i);
}

void writeMHzToLCD(uint32_t i) {
#if CAPTURE_MULT == 10000
  if((i/1000000) < 10)
    writeCharToLCD(' ');
  writeDecToLCD(i/1000000);
  writeCharToLCD(',');
  if(((i%1000000)/1000) < 100)
    writeCharToLCD('0');
  if(((i%1000000)/1000) < 10)
    writeCharToLCD('0');
  writeDecToLCD((i%1000000)/1000);
  writeCharToLCD(',');
  if((i%1000) < 100)
    writeCharToLCD('0');
  if((i%1000) < 10)
    writeCharToLCD('0');
  writeDecToLCD(i%1000);
#endif
#if CAPTURE_MULT == 50000
  uint32_t j = i/5;
  uint8_t k = 8;
  while((j>9) && k)
  {
    j /= 10;
    --k;
  }
  while(--k)
  {
    writeCharToLCD(' ');
  }
  writeDecToLCD(i/5);
  writeCharToLCD(',');
  writeDecToLCD(2*(i%5));
#endif
  writeStringToLCD("MHz");
}

void writeQ88ToLCD(uint16_t i) {
  char buff[7];
  char *p = &buff[0];
  strprintf(&p, "%q", i);
  *p = 0;
  writeStringToLCD(buff);
}

void writeQ4CToLCD(uint16_t i) {
  char buff[7];
  char *p = &buff[0];
  strprintf(&p, "%r", i);
  *p = 0;
  writeStringToLCD(buff);
}

void writeByteToLCD(uint8_t i) {
  writeCharToLCD(i2h(i>>4));
  writeCharToLCD(i2h(i));
}

void writeWordToLCD(uint16_t i) {
  writeByteToLCD(i>>8);
  writeByteToLCD(i&0x00FF);
}

void writeStringToLCD(const char *string) {
    while(*string) {
        writeCharToLCD(*string++);
    }
}

void clearLCD() {
    setAddr(0, 0);
    int i = 0;
    while(i < PCD8544_MAXBYTES) {
        writeToLCD(LCD5110_DATA, 0);
        i++;
    }
    setAddr(0, 0);
}

void pixel(uint8_t x, uint8_t y) {
  uint8_t row = y / 8;
  if(row > 4)
    return;
  if(row < 1)
    return;
  uint8_t line = 1 << (y%8);
  setAddr(x, row);
  writeToLCD(LCD5110_DATA, line);
};

void clearBank(unsigned char bank) {
  setAddr(0, bank);
  int i = 0;
  while(i < PCD8544_HPIXELS) {
      writeToLCD(LCD5110_DATA, 0);
      i++;
  }
  setAddr(0, bank);
}

void graph(uint8_t x, uint8_t y, uint8_t y0) {
  uint8_t row = y / 8, row0 = y0 / 8;
  if(row0 > (LCD_MAX_Y/8))
    return;
  uint8_t line = 1 << (y%8), line0 = 1 << (y0%8);
  if(row == row0) {
    if((x % 5) == 0)
      line ^= line0;
    setAddr(x, row);
    writeToLCD(LCD5110_DATA, line);
  } else {
    if((row > 0) && (row < 5)) {
      setAddr(x, row);
      writeToLCD(LCD5110_DATA, line);
    }
    if((x % 5) == 0) {
      setAddr(x, row0);
      writeToLCD(LCD5110_DATA, line0);
    }
  }
};

void bargraph(uint8_t row, uint16_t val) {
  if(row > (LCD_MAX_Y/8))
    return;
  if(prev_bar_val == val)
    return;
  if(prev_bar_val == 255) {
    uint8_t x = 0;
    setAddr(x, row);
    writeToLCD(LCD5110_DATA, 0xFF);
    while(++x < LCD_MAX_X-1) {
      setAddr(x, row);
      writeToLCD(LCD5110_DATA, (x<val)?0xFF:0x81);
    }
    writeToLCD(LCD5110_DATA, 0xFF);
  } else {
    if(prev_bar_val < val) {
      setAddr(prev_bar_val, row);
      while(prev_bar_val < val) {
        writeToLCD(LCD5110_DATA, 0xFF);
        ++prev_bar_val;
      }
    } else {
      uint8_t x = prev_bar_val;
      setAddr(x, row);
      while(x < prev_bar_val) {
        writeToLCD(LCD5110_DATA, 0x81);
        ++x;
      }
      prev_bar_val = val;
    }
  }
};

#define BOT  0x80
#define BOTM 0x70
#define TOP  0x01
#define TOPM 0x0E
#define MID  0xAA
#define FLANK 0x7E

//                    where?       rising edge   extra marker
void phase_difference(uint8_t row, uint16_t val, uint16_t marker) {
  if((val == prev_pd_val) || (row > (LCD_MAX_Y/8)))
    return;
  uint16_t valf = (val + (LCD_MAX_X/2)) % LCD_MAX_X;
  if(prev_pd_val == 255) {
    uint8_t x = 0;
    setAddr(x, row);
    while(x < LCD_MAX_X) {
      //setAddr(x, row);
      if(x == 42) {
        writeToLCD(LCD5110_DATA, MID);
      } else {
        if((x == val) || (x == valf)) {
          writeToLCD(LCD5110_DATA, (x==marker)?0x66:FLANK);
        } else
          if(val < valf) {
            //     _____
            // ___|     |____
            writeToLCD(LCD5110_DATA, ((x>val)&&(x<valf))?
              ((x==marker)?TOPM:TOP)
              :
              ((x==marker)?BOTM:BOT));
          } else {
            // ___        ___
            //    |______|
            writeToLCD(LCD5110_DATA, ((x<valf)||(x>val))?
              ((x==marker)?TOPM:TOP)
              :
              ((x==marker)?BOTM:BOT));
          }
      }
      ++x;
    }
  } else {
    // move to the right
    if(prev_pd_val < val) {
      setAddr(prev_pd_val, row);
      writeToLCD(LCD5110_DATA, BOT);
      while(prev_pd_val < val-1) {
        writeToLCD(LCD5110_DATA, (prev_pd_val==42)?MID:((prev_pd_val==marker)?0x0F:BOT));
        ++prev_pd_val;
      }
      writeToLCD(LCD5110_DATA, (prev_pd_val==marker)?0x66:FLANK);
      // falling edge
      setAddr(prev_pd_valf, row);
      writeToLCD(LCD5110_DATA, TOP);
      while(prev_pd_valf < valf-1) {
        writeToLCD(LCD5110_DATA, (prev_pd_valf==42)?MID:((prev_pd_valf==marker)?0x0F:TOP));
        ++prev_pd_valf;
      }
      writeToLCD(LCD5110_DATA, BOT);
    } else {
      // move to the left
      uint8_t x = val;
      writeToLCD(LCD5110_DATA, (x==marker)?0x66:FLANK);
      while(x < prev_pd_val-1) {
        writeToLCD(LCD5110_DATA, (x==42)?MID:((x==marker)?BOTM:BOT));
        ++x;
      }
      writeToLCD(LCD5110_DATA, ((x==marker)?TOPM:TOP));
      // falling edge
      x = valf;
      writeToLCD(LCD5110_DATA, (x==marker)?0x66:FLANK);
      while(x < prev_pd_valf-1) {
        writeToLCD(LCD5110_DATA, (x==42)?MID:((x==marker)?BOTM:TOP));
        ++x;
      }
      writeToLCD(LCD5110_DATA, ((x==marker)?TOPM:BOT));
    }
    prev_pd_valf = valf;
  }
};
