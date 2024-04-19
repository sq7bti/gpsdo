//***************************************************************************************
//  MSP430 + PCD8544-based LCD (like Nokia 5110)
//
//  MSP430x2xx Family User's Guide      : http://www.ti.com/lit/ug/slau144j/slau144j.pdf
//  MSP430G2x53 Data Sheet              : http://www.ti.com/lit/ds/symlink/msp430g2553.pdf
//  PCD8544 Data Sheet (Nokia 5110 MCU) : https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
//
//  My setup:
//
//         NOKIA 5110 LCD                                               MSP-EXP430G2
//       -----------------                                           -------------------
//      |              GND|<-- Ground ------------------------------|J6     GND         |
//      |               BL|<-- Back-light (not connected)
//      |              VCC|<-- Vcc +3..5V --------------------------|J1.1   VCC         |
//      |                 |
//      |              CLC|<-- Clock -------------------------------|J1.7   P1.5        |
//      |              DIN|<-- Data Input --------------------------|J2.15  P1.7        |
//      |               DC|<-- Data/Command (high/low) -------------|J2.13  P2.5        |
//      |               CE|<-- Chip Enable (active low) ------------|J1.8   P2.0        |
//      |              RST|<-- Reset -------------------------------|J2.16  RST
//
//
//  This example is based on the RobG's example : http://forum.43oh.com/topic/1312-nokia-5110-display
//  Changes:
//  - Removed graphics function
//  - Removed unused functions
//  + LCD test runs in the loop
//  + Added some bars animation
//
//***************************************************************************************

#include <msp430g2553.h>
#include "PCD8544.h"
#include "USCI_A0_uart.h"


void initCPU(void){
	  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	  DCOCTL = 0;                               // Run at 16 MHz
	  BCSCTL1 = CALBC1_16MHZ;                   //
	  DCOCTL  = CALDCO_16MHZ;                   //
	  BCSCTL2 = SELM_0;							// use DCO as system clock (MCLK)
	  BCSCTL1 |= DIVA_0;                        // ACLK/0
	  BCSCTL3 |= XCAP_3;						//12.5pF cap- setting for 32768Hz crystal
}

void initSPI(void) {

  P2OUT |= LCD5110_SCE_PIN | LCD5110_DC_PIN | LCD5110_BL_PIN | LCD5110_RST_PIN;  // Disable LCD, set Data mode
  P2DIR |= LCD5110_SCE_PIN | LCD5110_DC_PIN | LCD5110_BL_PIN | LCD5110_RST_PIN;  // Set pins to output direction

  // Setup USIB
  P1SEL |= LCD5110_SCLK_PIN | LCD5110_DN_PIN;
  P1SEL2 |= LCD5110_SCLK_PIN | LCD5110_DN_PIN;

  UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC; // 3-pin, 8-bit SPI master
  UCB0CTL1 |= UCSSEL_2;               // SMCLK
  UCB0BR0 |= 0x01;                    // 1:1
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;               // clear SW
}

int main(void) {

    initCPU();
    initUART();
    initSPI();

    int c, i;

    for(i = 16; i > 0; --i)
      __delay_cycles(500000);

    initLCD();
    clearLCD();

    LCD5110_BL_ON;
    for(i = 16; i > 0; --i)
      __delay_cycles(500000);

    // it was power on so perform full reset of GPS module
    setAddr(10, 2);
    if((IFG1 & RSTIFG)) {
      writeStringToLCD("RSTIFG set");
    } else {
      writeStringToLCD("RSTIFG clear");
      puts("$PRWIINIT,A,,,,,,,,,,,,000000,160424");
      putc('\r');
      putc('\n');
    }

    for(i = 16; i > 0; --i)
      __delay_cycles(2000000);


    while(1) {
        clearLCD();
        setAddr(0, 0);

        if(new_frame()) {
          ack_frame();
          for(i = 0; i < 5; i++) {
            clearBank(i);
            setAddr(0, i);
            for(c = 0; c < 14; c++) {
              if(rxBuf()[c + (i*14)] == '\r')
                break;
              writeCharToLCD(rxBuf()[c + (i*14)]);
            }
          }
          setAddr(84 - 12, 5);
          writeCharToLCD(dtohex(checksum() >> 4));
          writeCharToLCD(dtohex(checksum() & 0x0F));
        }

//        for(i = 16; i > 0; --i)
          __delay_cycles(20000000);
    } // eof while()
} // eof main
