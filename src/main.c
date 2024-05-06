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

#define KP +6
#define KI -4
#define KD +6

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

typedef enum {
  STARTUP = 0,
  WARM_UP = 1,
  LOCKING = 2,
  TRACKING = 3
} ctrl_state_t;

const char ctrl_state_name[] = { 's', 'w', 'l', 't'};

void pid_controller() {
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

const char warning[] = "start tracking; target phase diff ";

ctrl_state_t controller(ctrl_state_t current_state) {
  ctrl_state_t next_state = current_state;
  switch(current_state) {
    case STARTUP:
      next_state = WARM_UP;
    break;
    case WARM_UP:
      if(getOCXOTemperature() > (47<<8))
        next_state = LOCKING;
    break;
    case LOCKING:
      if((fix_status == 'A') && ref_tracked && vco_tracked) {
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
        putstring(warning);
        // target phaes diff should be <-999 ... +999
        char t_p_d[6];
        char *p = &t_p_d[0];
        while(txBusy());
        strprintf(&p, "%i\r\n", target_phase_diff);
        *p = 0;
        putstring(t_p_d);
        next_state = TRACKING;
      }
    break;
    case TRACKING:
      pid_controller();
      if(fix_status != 'A')
        next_state = LOCKING;
    break;
  }
  return next_state;
}
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

  // it was power on so perform full reset of GPS module
  setAddr(10 + (6*5), 2);
  writeStringToLCD("start");
  setAddr(10, 2);
  if(IFG1 & RSTIFG) {
    writeStringToLCD("warm");
  } else { //             012345678901234
    char nmea_enable[] = "$PRWIILOG,???,V,,,\r\n";
    writeStringToLCD("cold");
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

    //nmea_enable[11] = 'S'; nmea_enable[12] = 'V';
    //putstring(nmea_enable);
    //while(txBusy());

    //nmea_enable[12] = 'A';
    //putstring(nmea_enable);
    //while(txBusy());

    //nmea_enable[10] = 'V'; nmea_enable[11] = 'T'; nmea_enable[12] = 'G';
    //putstring(nmea_enable);
  }

  i = getSeconds() + 3;
  while(getSeconds() < i);

  clearLCD();
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
  uint8_t x = 0, y = 0;

  setup();
  clearBank(1);
  clearBank(2);
  clearBank(3);

  ctrl_state_t gpsdo_ctrl_state = STARTUP;

  while(1) {
    // execute to update fast changing values
    if(getTrigFlag(TRIGGER_LCD)) {
      if(gpsdo_ctrl_state == TRACKING) {
#ifdef OVERCLOCK
        phase_difference(3, (phase_diff + 1000) / 24);
#else
        phase_difference(3, (phase_diff + 800) / 20);
#endif /* OVERCLOCK */
      }
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
      //strprintf(&p, "%c%i T%q %x F:%l E:%i P:%i I:%i D:%i\r\n",
      //              fix_status, used_sats, getOCXOTemperature(), getPWM(),
      //              getOCXO() - 10000000UL, error_current,
      //              p_factor, i_factor, d_factor);
      strprintf(&p, "F%c%i T%q POT %x = %rV %l\r\n",
              fix_status, used_sats, getOCXOTemperature(), getPhaseDet(), getPhaseDet(), getOCXO());
      //strprintf(&p, "Int %q Oven %q ADC %r\r\n",
      //            getIntTemperature(), getOCXOTemperature(), getPhaseDet());
      *p = 0;
      putstring(monitoring);
      clearTrigFlag(TRIGGER_LOG);
    }

    // executed once a second (time!)
    if(getTrigFlag(TRIGGER_SEC)) {

      //if(fix_status == 'A') {
      if(gpsdo_ctrl_state == TRACKING) {
        //setAddr(0, 2);
        //clearBank(2);

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

        //writeStringToLCD("p");
        //if(p_factor != 0)
        //  writeCharToLCD(p_factor > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(p_factor));

        //writeStringToLCD("i");
        //if(i_factor != 0)
        //  writeCharToLCD(i_factor > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(i_factor));

        //writeStringToLCD("d");
        //if(d_factor != 0)
        //  writeCharToLCD(d_factor > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(d_factor));
      }

      //if(gpsdo_ctrl_state == LOCKING) {
      //  writeMHzToLCD(getOCXO());
      //}

      //setAddr(84 - (5 * 6), 4);
      //writeDecToLCD(gpsdo_ctrl_state);
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

      writeCharToLCD(' ');
      //writeCharToLCD(' ');

      setInverse(time_upd);
      writeCharToLCD('T');
      setInverse(FALSE);

      setInverse(date_upd);
      writeCharToLCD('D');
      setInverse(FALSE);

      setInverse(latitude_upd | longitude_upd);
      writeCharToLCD('P');
      setInverse(FALSE);

      writeCharToLCD(' ');
      writeCharToLCD(ctrl_state_name[gpsdo_ctrl_state]);

      //if(fix_status_upd || used_sats_upd) {
        writeCharToLCD(fix_status);
        writeCharToLCD(i2h(used_sats));
        fix_status_upd = FALSE;
        used_sats_upd = FALSE;
      //}

      if(gpsdo_ctrl_state == TRACKING) {
        //setAddr(0, 1);
        //clearBank(1);

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

        //if(target_phase_diff != 0)
        //  writeCharToLCD(target_phase_diff > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(target_phase_diff));
        //writeCharToLCD(' ');

        //writeQ4CToLCD(getPhaseDet());
        //writeCharToLCD('V');
        //writeCharToLCD(' ');
        //writeWordToLCD(getPWM());

        //if(phase_driftrate != 0)
        //  writeCharToLCD(phase_driftrate > 0?'+':'-');
        //else
        //  writeCharToLCD(' ');
        //writeDecToLCD(abs(2 * phase_driftrate));
        //writeStringToLCD("Hz");
      }

      if(gpsdo_ctrl_state == WARM_UP) {
        // draw in lines 1 and 2 (8 .. 23) - range 16,
        // temperature changes from ~20 .. 47 - range 27
        // getOCXOTemperature -> Q88
        y = 24 - ((getOCXOTemperature() - (20 << 8)) >> 9);
        setAddr(x, 1);
        writeToLCD(LCD5110_DATA, 0);
        setAddr(x, 2);
        writeToLCD(LCD5110_DATA, 0);
        pixel(x,y);
        ++x;
        if(x > 84)
          x = 0;
        setAddr(x, 1);
        writeToLCD(LCD5110_DATA, 0xFF);
        setAddr(x, 2);
        writeToLCD(LCD5110_DATA, 0xFF);
      }
      if(gpsdo_ctrl_state == TRACKING) {
        y = 16 - (error_current); ///1); //(int8_t)((getPWM() - TA0CCR1_DEF) >> 3);
        setAddr(x, 1);
        writeToLCD(LCD5110_DATA, 0);
        setAddr(x, 2);
        writeToLCD(LCD5110_DATA, 0);
        pixel(x,y);
        ++x;
        if(x > 84)
          x = 0;
        setAddr(x, 1);
        writeToLCD(LCD5110_DATA, 0xFF);
        setAddr(x, 2);
        writeToLCD(LCD5110_DATA, 0xFF);

        setAddr(84 - 6*4, 4);
        //writeWordToLCD(getPWM() - TA0CCR1_DEF);
        //writeStringToLCD("e");
        if(error_current != 0)
          writeCharToLCD(error_current > 0?'+':'-');
        else
          writeCharToLCD(' ');
        writeDecToLCD(abs(error_current));
      }
      if(gpsdo_ctrl_state == LOCKING) {
        setAddr(0, 3);
        clearBank(3);
#if 1
        char sbuff[15];
        char *t = &sbuff[0];
        strprintf(&t, "%rV %q%cC", getPhaseDet(), getIntTemperature(), 0x7F);
        *t = 0;
        writeStringToLCD(sbuff);
#else
        writeQ4CToLCD(getPhaseDet());
        writeCharToLCD('V');
        writeCharToLCD(' ');
        writeQ88ToLCD(getIntTemperature());
        writeCharToLCD(0x7F);
        writeCharToLCD('C');
#endif
      }
      clearTrigFlag(TRIGGER_ADC);
    }

    if(!getticks()) {
      gpsdo_ctrl_state = controller(gpsdo_ctrl_state);
    }

    LPM0;
  } // eof while()
} // eof main
