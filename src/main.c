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
//      |               BL|<-- Back-light - tie to ground via res   |                   |
//      |              VCC|<-- Vcc +3..5V --------------------------|1      VCC         |
//      |                 |                                         |                   |
//      |              CLC|<-- Clock -------------------------------|7      P1.5        |
//      |              DIN|<-- Data Input --------------------------|15     P1.7        |
//      |               DC|<-- Data/Command (high/low) -------------|11     P2.3        |
//      |               CE|<-- Chip Enable (active low) ------------|18     P2.7  XOUT  |
//      |              RST|<-- Reset - RC                           |                   |
//       -----------------                                          |                   |
//                                                                  |                   |
//              GPS                                                 |                   |
//       -----------------                                          |                   |
//      |              TX |<-- NMEA output -------------------------|3      P1.1        |
//      |              RX |<-- NMEA input --------------------------|4      P1.2        |
//      |           10kHz |<-- GPS reference signal--------+--------|19     P2.6 XIN    |
//      |                 |                                \--------|9      P2.1        |
//      |             PPS |<-- NMEA input --------------------------|Jx.x   Px.x        |
//       -----------------                                          |                   |
//                                                                  |                   |
//              OCXO                                                |                   |
//       -----------------                                          |                   |
//      |           10MHz |<-- VCO output --------------------------|2      P1.0 (ext)  |
//      |           10kHz |<-- VCO output divided % 1000 -----------|12     P2.4        |
//      |            Vref |<-- VCO input ------------\--------o<|---|14     P1.6 (neg)  |
//      |                 |                           \ ------o<|---|14     P2.0 (neg)  |
//      |            LM35 |<-- temperature -------------------------|5      P1.3        |
//       -----------------                                          |                   |
//                                                                  |                   |
//              4046                                                |                   |
//       -----------------                                          |                   |
//      |              PC |<-- Phase comparator output -------------|6      P1.4        |
//       -----------------                                           -------------------
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
#include "timer.h"
#include "adc.h"
#include "printf.h"

#define NMEA_RMC_LOCK 23
#define NMEA_RMC_TIM 6
#define NMEA_RMC_LAT 25
#define NMEA_RMC_LNG 27

#define KP +3
#define KI -4
#define KD 0

// latitude
// 0123456789A
// 1234.5678,N
// 12*34'5678N
// 0123456789A
// 013455667AA
// 2-> degrees
// 5-> apostrophe
//                     0  1  2  3  4  5  6  7  8  9  10
//const int8_t lat_array[] = { 0, 1, 3, 4,-1, 6, 7, 8, 9,-1, 11 }; // 11
// longitude
// 0123456789AB
// 01234.5678,E
// 012*34'5678E
// 0124566789AA
//                     0  1  2  3  4  5  6  7  8  9 10, 11
//const int8_t lon_array[] = { 0, 1, 2, 4, 5,-1, 7, 8, 9,10,-1, 12 }; // 12
// 3 -> degrees
// 4 -> apostrophe

/* WDT is clocked by fACLK (assumed 10kHz) */
#define WDT_XDLY_3267       (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL)                 /* 3267.8ms  " /32768 */
#define WDT_XDLY_819        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS0)          /* 819.2ms   " /8192  */
#define WDT_XDLY_51         (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1)          /* 51.2ms    " /512   */
#define WDT_XDLY_6_4        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0)   /* 6.4ms   " /64    */

#define DCO_0               ()
#define DCO_1               (DCO0)
#define DCO_2               (DCO1)
#define DCO_3               (DCO1+DCO0)
#define DCO_4               (DCO2)
#define DCO_5               (DCO2+DCO0)
#define DCO_6               (DCO2+DCO1)
#define DCO_7               (DCO2+DCO1+DCO0)

void initCPU(void){
	  //WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    WDTCTL = WDT_XDLY_6_4;
    //WDTCTL = WDT_XDLY_51;
    //WDTCTL = WDT_MDLY_0_5;                    //equivalent to half millisecond
    IE1 |= WDTIE;                             // Enable WDT interrupt}

	  DCOCTL = 0;                               // Run at 16 MHz
	  BCSCTL1 = CALBC1_16MHZ;                   //
#ifndef OVERCLOCK
    DCOCTL  = CALDCO_16MHZ;                   //
#else
    DCOCTL  = CALDCO_16MHZ & ~DCO_7;                   //
    //DCOCTL &= DCO_7;                          // overclock from 16MHz +
    //DCOCTL |= DCO_4;                          // DCO_4 - ~16.2MHz
    //DCOCTL |= DCO_5;                          // DCO_5 - ~17.5MHz
    //DCOCTL |= DCO_6;                          // DCO_6 - ~19.2MHz
    DCOCTL |= DCO_7;                          // DCO_7 - ~20.0MHz
#endif /* OVERCLOCK */
	  BCSCTL2 = SELM_0;							            // use DCO as system clock (MCLK)
	  BCSCTL1 |= DIVA_0;                        // ACLK/0
	  BCSCTL3 |= LFXT1S_3;						            //12.5pF cap- setting for 32768Hz crystal
}

void initSPI(void) {

  P2OUT |= LCD5110_DC_PIN;  // Disable LCD, set Data mode
  P2DIR |= LCD5110_DC_PIN;  // Set pins to output direction

  P2OUT |= LCD5110_SCE_PIN;  // Disable LCD, set Data mode
  P2DIR |= LCD5110_SCE_PIN;  // Set pins to output direction
  P2SEL &= ~LCD5110_SCE_PIN;  // Set pins to output direction
  P2SEL2 &= ~LCD5110_SCE_PIN;  // Set pins to output direction

  // Setup USIB
  P1SEL |= LCD5110_SCLK_PIN | LCD5110_DN_PIN; // | LCD5110_SCE_PIN;
  P1SEL2 |= LCD5110_SCLK_PIN | LCD5110_DN_PIN; // | LCD5110_SCE_PIN;
  UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC; // 3-pin, 8-bit SPI master

  UCB0CTL1 |= UCSSEL_2;               // SMCLK
  UCB0BR0 |= 0x01;                    // 1:1
  UCB0BR1 = 0;
  UCB0CTL1 &= ~UCSWRST;               // clear SW
}

extern int16_t phase_diff, phase_driftrate;
int16_t target_phase_diff = 32000;
uint16_t temp_phase_diff;

extern bool vco_tracked;
extern bool ref_tracked;

extern uint8_t txCount;

int16_t error_current, error_previous;
int16_t p_factor;
int32_t i_factor;
int16_t d_factor;

int setup(void) {
  initCPU();
  initUART();
  initSPI();
  initTIMER();
  initADC();

  __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled

  int i = 0;

  for(i = 16; i > 0; --i)
    __delay_cycles(500000);

  initLCD();
  clearLCD();

  for(i = 16; i > 0; --i)
    __delay_cycles(500000);

//  if(IFG1 & OFIFG)
//    writeStringToLCD("Xtal fault");

/*  writeStringToLCD("16 R:");
  uint8_t calbc_16 = (uint16_t*)CALBC1_16MHZ;
  writeDecToLCD(0x0F & calbc_16);
  writeStringToLCD(" D:");
  uint8_t caldc_16 = (uint16_t*)CALDCO_16MHZ;
  writeDecToLCD(caldc_16 >> 5);
  setAddr(0, 1);
  writeStringToLCD("12 R:");
  uint8_t calbc_12 = (uint16_t*)CALBC1_12MHZ;
  writeDecToLCD(0x0F & calbc_12);
  writeStringToLCD(" D:");
  uint8_t caldc_12 = (uint16_t*)CALDCO_12MHZ;
  writeDecToLCD(caldc_12 >> 5);
  setAddr(0, 2);
  writeStringToLCD(" 8 R:");
  uint8_t calbc_8 = (uint16_t*)CALBC1_8MHZ;
  writeDecToLCD(0x0F & calbc_8);
  writeStringToLCD(" D:");
  uint8_t caldc_8 = (uint16_t*)CALDCO_8MHZ;
  writeDecToLCD(caldc_8 >> 5);
  setAddr(0, 3);
  writeStringToLCD(" 1 R:");
  uint8_t calbc_1 = (uint16_t*)CALBC1_1MHZ;
  writeDecToLCD(0x0F & calbc_1);
  writeStringToLCD(" D:");
  uint8_t caldc_1 = (uint16_t*)CALDCO_1MHZ;
  writeDecToLCD(caldc_1 >> 5);*/

  //dump_tlv_segment(0x10C0, 32);
//  while(1);

  // it was power on so perform full reset of GPS module
  setAddr(10, 2);
  if(IFG1 & RSTIFG) {
    writeStringToLCD("warm start");
  } else { //             012345678901234
    char nmea_enable[] = "$PRWIILOG,???,V,,,\r\n";
    writeStringToLCD("GPS Reset");
    putstring("$PRWIINIT,A,,,,,,,,,,,,000000,160424\r\n");
    while(txBusy());
    putstring(nmea_enable);
    while(txBusy());

    nmea_enable[14] = 'A'; nmea_enable[10] = 'R'; nmea_enable[11] = 'M'; nmea_enable[12] = 'C';
    putstring(nmea_enable);
    while(txBusy());

    nmea_enable[10] = 'G'; nmea_enable[11] = 'G'; nmea_enable[12] = 'A';
    putstring(nmea_enable);
    while(txBusy());

    nmea_enable[11] = 'S'; nmea_enable[12] = 'V';
    putstring(nmea_enable);
    while(txBusy());

    nmea_enable[12] = 'A';
    putstring(nmea_enable);
    while(txBusy());

    nmea_enable[10] = 'V'; nmea_enable[11] = 'T'; nmea_enable[12] = 'G';
    putstring(nmea_enable);
  }
//    for(i = 16; i > 0; --i)
    __delay_cycles(2000000);

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
/*    clearBank(2);
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
*/
}

#define NMEA_FIX 0

uint8_t nmea_valid = 0;
extern volatile bool rx_busy;
extern volatile uint8_t log_update;

bool direction = 1;

int main(void) {

  //                   0000000000111111111122222222223333333333444444444455555555556666666666
  //                   0123456789012345678901234567890123456789012345678901234567890123456789
  char monitoring[] = "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF\r\n";
  char *p = &monitoring[0];

  setup();
  clearBank(1);
  clearBank(2);
  clearBank(3);

  while(1) {
    // execute to update fast changing values
    if(getTrigFlag(TRIGGER_LCD)) {
#ifdef OVERCLOCK
      phase_difference(3, (phase_diff + 1000) / 24);
#else
      phase_difference(3, (phase_diff + 800) / 20);
#endif /* OVERCLOCK */

      //setAddr(84 - 6, 4);
      //setInverse(rx_busy);
      //writeCharToLCD('R');
      //setInverse(FALSE);
      //setAddr(84 - (3*6), 4);
      //writeDecToLCD(log_update);
      clearTrigFlag(TRIGGER_LCD);
    }

    // once the logging is synchronized with NMEA stream
    if(getTrigFlag(TRIGGER_LOG)) {
      p = &monitoring[0];
      while(txBusy());
      //strprintf(&p, "PWM %x ph %i P:%i I:%i D:%i\r\n", getPWM(), phase_diff, p_factor, i_factor, d_factor);
      //strprintf(&p, "%c%i T%q P%x %i E:%i P:%i I:%i D:%i\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getPWM(),
      //        getOCXO() - 10000000, error_current,
      //        p_factor, i_factor, d_factor);
      strprintf(&p, "%c%i T%q %x F:%l E:%i P:%i I:%i D:%i\r\n",
                    fix_status, used_sats, getOCXOTemperature(), getPWM(),
                    getOCXO() - 10000000UL, error_current,
                    p_factor, i_factor, d_factor);
      //strprintf(&p, "F%c%i T%q POT %x = %rV %l\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getPhaseDet(), getPhaseDet(), getOCXO());
      *p = 0;
      putstring(monitoring);
      clearTrigFlag(TRIGGER_LOG);
    }

    // executed once a second (time!)
    if(getTrigFlag(TRIGGER_SEC)) {
      setAddr(0, 2);
      clearBank(2);

      if(fix_status == 'A') {

        //if((getSeconds()%10) == 0) {
        //  target_phase_diff += 250;
        //  target_phase_diff %= 1000;
        //}

        //writeStringToLCD("e");
        //if(error_current != 0)
        //  writeCharToLCD(error_current > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(error_current));

        writeStringToLCD("p");
        if(p_factor != 0)
          writeCharToLCD(p_factor > 0?'+':'-');
        else
          writeCharToLCD(' ');
        writeDecToLCD(abs(p_factor));

        writeStringToLCD("i");
        if(i_factor != 0)
          writeCharToLCD(i_factor > 0?'+':'-');
        else
          writeCharToLCD(' ');
        writeDecToLCD(abs(i_factor));

        //writeStringToLCD("d");
        //if(d_factor != 0)
        //  writeCharToLCD(d_factor > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(d_factor));
      } else {
        writeMHzToLCD(getOCXO());
      }

      //if((getSeconds() % 20) == 0) {
      //  setAddr(0, 3);
      //  clearBank(3);
      //  writeStringToLCD(latitude);
      //}
      //if((getSeconds() % 20) == 10) {
      //  setAddr(0, 3);
      //  clearBank(3);
      //  writeStringToLCD(longitude);
      //}

      if(time_upd) {
        for(int c = 0; c < 8; ++c) {
          if(time_upd & (1<<c)) {
            setAddr(6 * c, 5);
            writeCharToLCD(time[c]);
            time_upd &= ~(1<<c);
          }
        }
      }
      if(date_upd) {
        for(int c = 0; c < 8; ++c) {
          if(date_upd & (1<<c)) {
            //setAddr(clock_array[c], 4);
            setAddr(6 * c, 4);
            writeCharToLCD(date[c]);
            date_upd &= ~(1<<c);
          }
        }
      }

      if(fix_status_upd || used_sats_upd) {
        setAddr(84 - (6 * 2), 5);
        writeCharToLCD(fix_status);
        writeCharToLCD(i2h(used_sats));
        fix_status_upd = FALSE;
        used_sats_upd = FALSE;
      }

      clearTrigFlag(TRIGGER_SEC);
    }

    // executed to follow ADC measurements
    if(getTrigFlag(TRIGGER_ADC)) {
      setAddr(0, 0);
      clearBank(0);
      setInverse(getOCXOTemperature() < (50<<8));
      writeQ88ToLCD(getOCXOTemperature());
      writeCharToLCD(0x7F);
      writeCharToLCD('C');
      setInverse(FALSE);
      writeQ88ToLCD(getIntTemperature());
      writeCharToLCD(0x7F);
      writeCharToLCD('C');

      setAddr(0, 1);
      clearBank(1);

      //writeQ4CToLCD(getPhaseDet());
      //writeCharToLCD('V');

      //writeWordToLCD(phase_comp_raw_value);
      //writeStringToLCD(" = 0x");
      //if(phase_diff != 0)
      //  writeCharToLCD(phase_diff > 0?'+':'-');
      //else
      //  writeCharToLCD(' ');
      //writeDecToLCD(abs(phase_diff));
      //writeCharToLCD(' ');

      if(target_phase_diff != 0)
        writeCharToLCD(target_phase_diff > 0?'+':'-');
      else
        writeCharToLCD(' ');
      writeDecToLCD(abs(target_phase_diff));
      writeCharToLCD(' ');

      //writeQ4CToLCD(getPhaseDet());
      //writeCharToLCD('V');
      //writeCharToLCD(' ');
      writeWordToLCD(getPWM());

      //if(phase_driftrate != 0)
      //  writeCharToLCD(phase_driftrate > 0?'+':'-');
      //else
      //  writeCharToLCD(' ');
      //writeDecToLCD(abs(2 * phase_driftrate));
      //writeStringToLCD("Hz");


      clearTrigFlag(TRIGGER_ADC);
    }

    if(!getticks()) {
      if((fix_status == 'A') && (getOCXOTemperature() > (50 << 8))) {
        if(target_phase_diff == 32000) {
          if(ref_tracked && vco_tracked) {
            target_phase_diff = phase_diff;
            if(target_phase_diff < -500)
              target_phase_diff = -500;
            if(target_phase_diff > 500)
                target_phase_diff = 500;
            if((target_phase_diff > -10) && (target_phase_diff < 10)) {
              if(phase_diff > 0)
                target_phase_diff = 10;
              else
                target_phase_diff = -10;
            }
            p = &monitoring[0];
            while(txBusy());
            strprintf(&p, "start tracking; target phase diff %i\r\n", target_phase_diff);
            *p = 0;
            putstring(monitoring);
          }
        } else {
          if(1) { //}(phase_diff < 1000) && (phase_diff > -1000)) {
            // track phase
            error_current = target_phase_diff - phase_diff;
            // +500 is angle of 90 degrees difference between REF and VCO
            // phase_diff <-1000 ... +1000>
            d_factor = (error_current - error_previous);
            error_previous = error_current;
            p_factor = error_previous;
            i_factor += error_previous >> 4;

            // p_factor <-500 .. + 1500>
            setPWM(TA0CCR1_DEF
                    + (p_factor << KP)
                    + (i_factor << KI)
                    + (d_factor << KD)
                  );

          }
        }
      }
    } //     if(!getticks()) {

    LPM0;
  } // eof while()
} // eof main
