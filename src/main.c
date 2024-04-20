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

#define NMEA_RMC_LOCK 23
#define NMEA_RMC_TIM 6
#define NMEA_RMC_LAT 25
#define NMEA_RMC_LNG 27

char lock_status;
// 01:23:45
//
uint8_t clock_array[] = { 0, 6, 18, 24, 36, 42 }; // 6
// latitude
// 0123456789A
// 1234.5678,N
// 12*34'5678N
// 0123456789A
// 013455667AA
// 2-> degrees
// 5-> apostrophe
//                     0  1  2  3  4  5  6  7  8  9  10
int8_t lat_array[] = { 0, 1, 3, 4,-1, 6, 7, 8, 9,-1, 11 }; // 11
// longitude
// 0123456789AB
// 01234.5678,E
// 012*34'5678E
// 0124566789AA
//                     0  1  2  3  4  5  6  7  8  9 10, 11
int8_t lon_array[] = { 0, 1, 2, 4, 5,-1, 7, 8, 9,10,-1, 12 }; // 12
// 3 -> degrees
// 4 -> apostrophe
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

    for(i = 16; i > 0; --i)
      __delay_cycles(500000);

    // it was power on so perform full reset of GPS module
    setAddr(10, 2);
    if((IFG1 & RSTIFG)) {
      writeStringToLCD("warm start");
    } else {
      writeStringToLCD("GPS Reset");
      putstring("$PRWIINIT,A,,,,,,,,,,,,000000,160424");
      sendchar('\r');
      sendchar('\n');
      for(i = 16; i > 0; --i)
        __delay_cycles(2000000);
      putstring("$PRWIILOG,GSV,V,,,");
      sendchar('\r');
      sendchar('\n');
    }
    for(i = 16; i > 0; --i)
      __delay_cycles(2000000);
    LCD5110_BL_ON;

    i = 0;
    // 01:23:45
    setAddr(12, 5);
    writeCharToLCD(':');
    setAddr(30, 5);
    writeCharToLCD(':');
    // 20-04:24
    setAddr(12, 4);
    writeCharToLCD('-');
    setAddr(30, 4);
    writeCharToLCD('-');
    clearBank(2);
    // 2-> degrees
    // 5-> apostrophe
    setAddr(2*6, 2);
    writeCharToLCD(0x7F);
    setAddr(5*6, 2);
    writeCharToLCD('\'');
    // 3 -> degrees
    // 6 -> apostrophe
    setAddr(3*6, 3);
    writeCharToLCD(0x7F);
    setAddr(6*6, 3);
    writeCharToLCD('\'');
    while(1) {
        //clearLCD();
        //setAddr(0, 0);

      if(new_frame) { //} && crc_good) {
        uint8_t rx_crc = 16 * h2i(rxbuffer[checksum_idx-1]) + h2i(rxbuffer[checksum_idx]);

//        if(crc_good) {
//      if(new_frame) {
        new_frame = FALSE;
        setAddr(84 - 6, 5);
        switch(msg_count % 4) {
          case 0: writeCharToLCD('-'); break;
          case 1: writeCharToLCD('/'); break;
          case 2: writeCharToLCD('|'); break;
          case 3: writeCharToLCD('\\'); break;
        }
        switch(frame_type) {
          case RMC:
            for(c = 0; c < 6; ++c) {
              if(time_upd & (1<<c)) {
                //setAddr(6 * c, 5);
                setAddr(clock_array[c], 5);
                writeCharToLCD(time[c]);
                time_upd &= ~(1<<c);
              }
            }
            for(c = 0; c < 6; ++c) {
              if(date_upd & (1<<c)) {
                setAddr(clock_array[c], 4);
                writeCharToLCD(date[c]);
                date_upd &= ~(1<<c);
              }
            }
            for(c = 0; c < 12; ++c) {
              if((latitude_upd & (1<<c)) && (lat_array[c] >= 0)) {
                setAddr(lat_array[c]*6, 2);
                writeCharToLCD(latitude[c]);
                latitude_upd &= ~(1<<c);
              }
            }
            //setAddr(0, 3);
            for(c = 0; c < 13; ++c) {
              if((longitude_upd & (1<<c)) && (lon_array[c]) >= 0) {
                setAddr(lon_array[c]*6, 3);
                writeCharToLCD(longitude[c]);
                longitude_upd &= ~(1<<c);
              }
            }
            if(fix_status_upd) {
              setAddr(84 - (6 * 4), 5);
              writeCharToLCD(fix_status);
            }
//              writeStringToLCD(time);
          break;
          case GGA:
            if(used_sats_upd) {
              setAddr(84 - (6 * 2), 5);
              writeCharToLCD(i2h(used_sats%10));
              if(used_sats > 10) {
                setAddr(84 - (6 * 3), 5);
                writeCharToLCD(i2h(used_sats/10));
              }
              used_sats_upd = FALSE;
            }
          break;
          default:
          break;
        }
//        if((frame_type == RMC) || ((frame_type == GSV) && (rxbuffer[8] == '1'))) {
//        if(frame_type == RMC) {
//          for(i; i; --i)
//            clearBank(i);
//          i = 0;
//        }
        switch(frame_type) {
          case RMC:
          case GGA:
          case GSA:
          case GSV:
          case VTG:
          //case UNKNOWN:
//              lock_status = rxbuffer[NMEA_RMC_LOCK];
//              for(c = NMEA_RMC_TIM; c < NMEA_RMC_TIM + 6; c++)
//                writeCharToLCD(rxbuffer[c]);
            break;
          default:
            clearBank(i);
            setAddr(0, i);
            for(c = 2; c < 6; c++)
              writeCharToLCD(rxbuffer[c]);
            // wait for the checksum to arrive at the uart
            // otherwise the previously received is still there
            //__delay_cycles(70000);
            //__delay_cycles(59650);

#if 0
            setAddr(84 - (1 * 6), i);
            if(crc_good)
              writeStringToLCD("+");
            else
              writeStringToLCD("-");
/*          if(((rxbuffer[checksum_idx-1]) == (i2h(checksum >> 4)))
        && ((rxbuffer[checksum_idx]) == (i2h(checksum & 0x0F))))
          writeStringToLCD(" OK");
        else
          writeStringToLCD("BAD");*/
#else
            setAddr(84 - (8 * 6), i);
            writeCharToLCD(i2h(bitTrack >> 4));
            writeCharToLCD(i2h(bitTrack & 0x0F));
//            writeCharToLCD(i2h(frame_type));
//            writeCharToLCD(' ');
            writeCharToLCD(rxbuffer[checksum_idx-1]);
            writeCharToLCD(rxbuffer[checksum_idx]);
            if(checksum == rx_crc)
              writeStringToLCD("=");
            else
              writeStringToLCD("!");
            if(crc_good)
              writeStringToLCD("=");
            else
              writeStringToLCD("!");
            writeCharToLCD(i2h(checksum >> 4));
            writeCharToLCD(i2h(checksum & 0x0F));
#endif
            ++i;
            if(i>2)
              i = 0;
            break;
          }
          crc_good = FALSE;
        }

//        for(i = 16; i > 0; --i)
//          __delay_cycles(20000000);

//        __bis_SR_register(LPM3_bits + GIE);
//        putstring("loop");

    } // eof while()
} // eof main
