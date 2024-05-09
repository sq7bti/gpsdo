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

#define KP (+6)
#define KI (-2)
#define KD (+4)

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

int16_t error_current, error_previous, error_max, error_min, error_delta;
int16_t p_factor;
int32_t i_factor;
int16_t d_factor;
uint16_t new_pwm;
uint8_t kp = KP, ki = KI, kd = KD;

uint16_t pid_loop_count = 0;
uint16_t correction_period = 1, correction_timer = 0, correction_margin = 32;

typedef enum {
  STARTUP = 0,
  WARM_UP = 1,
  LOCKING = 2,
  TRACKING = 3
} ctrl_state_t;

const char ctrl_state_name[] = { 's', 'w', 'l', 't'};

//                   A5 T50.0 561C F:-5 E:-240 P:-240 I:-5441 D:-1
//                   0000000000111111111122222222223333333333444444444455555555556666666666
//                   0123456789012345678901234567890123456789012345678901234567890123456789
char monitoring[] = "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF\r\n";
char *p = &monitoring[0];

const char warning[] = "start tracking; target phase diff ";

ctrl_state_t controller(ctrl_state_t current_state) {
  ctrl_state_t next_state = current_state;
  switch(current_state) {
    case STARTUP:
      next_state = WARM_UP;
    break;
    case WARM_UP:
      if((getOCXOTemperature() > ((fix_status == 'A')?(35 << 8):(48 << 8))))
        next_state = LOCKING;
    break;
    case LOCKING:
      if((fix_status == 'A') && ref_tracked && vco_tracked) {
        target_phase_diff = phase_diff;
        if(target_phase_diff < -650)
          target_phase_diff = -650;
        if(target_phase_diff > 650)
            target_phase_diff = 650;
        if((target_phase_diff > -50) && (target_phase_diff < 50)) {
          if(target_phase_diff > 0)
            target_phase_diff = 50;
          else
            target_phase_diff = -50;
        }
        // target phaes diff should be <-999 ... +999
        char *p = &monitoring[0];
        strprintf(&p, "start tracking; target phase diff %c%i ... \r\n", target_phase_diff>0?'+':' ', target_phase_diff);
        *p = 0;
        putstring(monitoring);
        while(txBusy());
        next_state = TRACKING;
        kp = KP;
        ki = KI;
        kd = KD;
        error_max = INT16_MIN;
        error_min = INT16_MAX;
      }
    break;
    case TRACKING:
      // track phase
      error_current = target_phase_diff - phase_diff;
      // +500 is angle of 90 degrees difference between REF and VCO
      // phase_diff <-1000 ... +1000>
      d_factor = (error_current - error_previous);
      error_previous = error_current;
      p_factor = error_previous;
      i_factor += error_previous >> 2;
      // p_factor <-500 .. + 1500>

      if(error_current > error_max)
        error_max = error_current;
      if(error_current < error_min)
        error_min = error_current;

      if(correction_timer)
        --correction_timer;
      else {
        correction_timer = correction_period;
        error_delta = error_max - error_min;
        error_max = INT16_MIN;
        error_min = INT16_MAX;
      }

      if(abs(error_current) < 64) {
        if(abs(error_current) < 16) {
          kp = KP-8;
          ki = KI-4;
          kd = KD-1;
        } else {
          kp = KP-5;
          ki = KI-3;
          kd = KD-1;
        }
      }

      new_pwm = TA0CCR1_DEF
                + (p_factor << kp)
                + (i_factor << ki);
      //          + (d_factor << KD);

      int16_t delta_pwm = new_pwm - getPWM();

      setPWM(getPWM() + delta_pwm);

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
  //setAddr(12, 5);
  //writeCharToLCD(':');
  //setAddr(30, 5);
  //writeCharToLCD(':');
  // 20-04:24
  //setAddr(12, 4);
  //writeCharToLCD('-');
  //setAddr(30, 4);
  //writeCharToLCD('-');
}

int main(void) {
  uint8_t x = 0, y = 0, y_min, y_max, y_scale = 4;

  setup();
  clearBank(1);
  clearBank(2);
  clearBank(3);

  ctrl_state_t gpsdo_ctrl_state = STARTUP;

  while(1) {

    // execute to update fast changing values
    if(getTrigFlag(TRIGGER_LCD)) {
      switch(gpsdo_ctrl_state) {
        case WARM_UP:
        case LOCKING:
          phase_difference(5, (phase_diff +
#ifdef OVERCLOCK
                                              1000
#else
                                              800
#endif /* OVERCLOCK */
          ) / 24, ((target_phase_diff==32000)?-1:(target_phase_diff +
#ifdef OVERCLOCK
                                                1000
#else
                                                800
#endif /* OVERCLOCK */
          ) / 24));
          //setAddr(0,1);
          //writeWordToLCD(phase_diff);
        break;
        case TRACKING:
          if(abs(error_current) > 32)
            phase_difference(5, (phase_diff +
#ifdef OVERCLOCK
                                              1000
#else
                                              800
#endif /* OVERCLOCK */
            ) / 24, (target_phase_diff +
#ifdef OVERCLOCK
                                          1000
#else
                                          800
#endif /* OVERCLOCK */
            ) / 24);
          else {
            setAddr(0, 5);
            writeMHzToLCD(getOCXO());
          }
          //setAddr(84 - (6*4), 1);
          setAddr(42 - (6*2), 1);
          writeWordToLCD(getPWM());
          writeCharToLCD(' ');

          //=================
          y = 24 - (error_current >> y_scale); ///1); //(int8_t)((getPWM() - TA0CCR1_DEF) >> 3);
          if(y > y_max)
            y_max = y;
          if(y < y_min)
            y_min = y;
          //y = 40 - (getPWM() >> 11);
          uint8_t x_sc = 84 - 6*((y_scale > 2)?4:3);
          if(x < x_sc) {
            setAddr(x, 1);
            writeToLCD(LCD5110_DATA, 0);
            setAddr(x, 4);
            writeToLCD(LCD5110_DATA, 0);
          }
          if(x == 0) {
            // 5 = +-512
            // 4 = +-256
            // 3 = +-128
            // 2 = +-64
            // 1 = +-32
            // 0 = +-16
            // y_max = 24 - E -> E = 24 - y_max
            setAddr(x_sc, 1);
            writeCharToLCD('+');
            writeDecToLCD((1 << (4 + y_scale)) - 1);
            setAddr(x_sc, 4);
            writeCharToLCD('-');
            writeDecToLCD((1 << (4 + y_scale)));
          }

          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0);

          if(((x > x_sc) && (y > 16) && (y < 32)) || (x < x_sc))
            pixel(x,y);
          if((x%5) == 0)
            pixel(x,24);
          ++x;
          if(x > 84) {
            // 8 .. 15
            // 16 .. 23
            // 24 .. 31
            // 32 .. 39
            if((y_scale != 0) && (y_max < 32) && (y_min > 16))
              --y_scale;
            if((y_scale < 5) && ((y_max > 39) || (y_min < 8)))
              ++y_scale;
            y_min = 40;
            y_max = 8;
            x = 0;
          }
          if(x < x_sc) {
            setAddr(x, 1); writeToLCD(LCD5110_DATA, 0xFF);
            setAddr(x, 4); writeToLCD(LCD5110_DATA, 0xFF);
          }
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0xFF);
          //=================
        break;
        default:
        break;
      }
      clearTrigFlag(TRIGGER_LCD);
    }

    // once the logging is synchronized with NMEA stream
    if(getTrigFlag(TRIGGER_LOG)) {
      p = &monitoring[0];
      while(txBusy());
      //strprintf(&p, "PWM %x ph %i P:%i I:%i D:%i\r\n", getPWM(), phase_diff, p_factor, i_factor, d_factor);
      //strprintf(&p, "%c%i T%q P%x %i E:%i P:%i I:%l D:%i\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getPWM(),
      //        getOCXO() - 10000000, error_current,
      //        p_factor, i_factor, d_factor);
#if CAPTURE_MULT == 10000
      strprintf(&p, "%c%i T%q %x F%l E%i eD%i P%i I%l D%i\r\n",
#endif
#if CAPTURE_MULT == 50000
      strprintf(&p, "%c%i T%q %x F%q E%i eD%i P%i I%l D%i\r\n",
#endif
                    fix_status, used_sats, getOCXOTemperature(), getPWM(),
#if CAPTURE_MULT == 10000
                    getOCXO() - 10000000UL,
#endif
#if CAPTURE_MULT == 50000
                    ((int16_t)(getOCXO() - 50000000UL) << 8)/5,
#endif
                    error_current, error_delta,
                    p_factor, i_factor, d_factor);
      //strprintf(&p, "F%c%i T%q POT %x = %rV %l\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getPhaseDet(), getPhaseDet(), getOCXO());
      //strprintf(&p, "Int %q Oven %q ADC %r\r\n",
      //            getIntTemperature(), getOCXOTemperature(), getPhaseDet());
      *p = 0;
      putstring(monitoring);
      clearTrigFlag(TRIGGER_LOG);
    }

    // executed once a second (time!)
    if(getTrigFlag(TRIGGER_SEC)) {
      switch(gpsdo_ctrl_state) {
        case TRACKING:
        break;
        case WARM_UP:
        //case LOCKING:
          // draw in lines 1 through 4 (8 .. 40) - range 32,
          // temperature changes from ~20 .. 52 - range 32
          // getOCXOTemperature -> Q88
          y = 40 - ((getOCXOTemperature() - (20 << 8)) >> 8);
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0);
          pixel(x,y);
          ++x;
          if(x > 84)
            x = 0;
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0xFF);
        break;
        default:
        break;
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
      if(gpsdo_ctrl_state == TRACKING)
        setInverse(abs(error_current) < 32);
      writeCharToLCD(ctrl_state_name[gpsdo_ctrl_state]);
      setInverse(FALSE);

      //if(fix_status_upd || used_sats_upd) {
        writeCharToLCD(fix_status);
        writeCharToLCD(i2h(used_sats));
        fix_status_upd = FALSE;
        used_sats_upd = FALSE;
      //}

      switch(gpsdo_ctrl_state) {
        case TRACKING:
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
        break;
        case WARM_UP:
        break;
        case LOCKING:
        {
          y_scale = 4;
          //setAddr(0, 3);
          //clearBank(3);
#if 1
          //char sbuff[15];
          //char *t = &sbuff[0];
          //strprintf(&t, "%rV %q%cC", getPhaseDet(), getIntTemperature(), 0x7F);
          //*t = 0;
          //writeStringToLCD(sbuff);
#else
          //writeQ4CToLCD(getPhaseDet());
          //writeCharToLCD('V');
          //writeCharToLCD(' ');
          //writeQ88ToLCD(getIntTemperature());
          //writeCharToLCD(0x7F);
          //writeCharToLCD('C');
#endif
        }
        break;
        default:
        break;
      }

      clearTrigFlag(TRIGGER_ADC);
    }

    if(!getticks()) {
      gpsdo_ctrl_state = controller(gpsdo_ctrl_state);
    }

    if(0) //!getTrigFlag(TRIGGER_ANY))
      LPM0;
  } // eof while()
} // eof main
