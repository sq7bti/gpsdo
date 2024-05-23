//***************************************************************************************
//  MSP430 + PCD8544-based LCD (like Nokia 5110)
//
//  MSP430x2xx Family User's Guide      : http://www.ti.com/lit/ug/slau144j/slau144j.pdf
//  MSP430G2x53 Data Sheet              : http://www.ti.com/lit/ds/symlink/msp430g2553.pdf
//  PCD8544 Data Sheet (Nokia 5110 MCU) : https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
//
//  My setup: 0123456789ABCD
//
//       NOKIA 5110 LCD  84x48 pixels                                              MSP-EXP430G2
//       --------------------------                                           -------------------
//      |  0123456789ABCD       GND|<-- Ground ------------------------------|J6     GND         |
//      |  0123456789ABCD        BL|<-- Back-light - tie to ground via res   |                   |
//      |  0123456789ABCD       VCC|<-- Vcc +3..5V --------------------------|1      VCC         |
//      |  0123456789ABCD          |                                         |                   |
//      |  0123456789ABCD       CLC|<-- Clock -------------------------------|7      P1.5        |
//      |                       DIN|<-- Data Input --------------------------|15     P1.7        |
//      |                        DC|<-- Data/Command (high/low) -------------|11     P2.3        |
//      |                        CE|<-- Chip Enable (active low) ------------|18     P2.7  XOUT  |
//      |                       RST|<-- Reset - RC                           |                   |
//       --------------------------                                          |                   |
//                                                                           |                   |
//                       GPS                                                 |                   |
//       --------------------------                                          |                   |
//      |                       TX |<-- NMEA output -------------------------|3      P1.1        |
//      |                       RX |<-- NMEA input --------------------------|4      P1.2        |
//      |                    10kHz |<-- GPS reference signal--------+--------|19     P2.6 XIN    |
//      |                          |                                \--------|9      P2.1        |
//      |                      PPS |<-- NMEA input --------------------------|Jx.x   Px.x        |
//       --------------------------                                          |                   |
//                                                                           |                   |
//                       OCXO                                                |                   |
//       --------------------------                                          |                   |
//      |                    10MHz |<-- VCO output --------------------------|2      P1.0 (ext)  |
//      |                    10kHz |<-- VCO output divided % 1000 -----------|12     P2.4        |
//      |                     Vref |<-- VCO input ------------\--------o<|---|14     P1.6 (neg)  |
//      |                          |                           \ ------o<|---|14     P2.0 (neg)  |
//      |                     LM35 |<-- temperature -------------------------|5      P1.3        |
//       --------------------------                                          |                   |
//                                                                           |                   |
//                       4046                                                |                   |
//       --------------------------                                          |                   |
//      |                       PC |<-- Phase comparator output -------------|6      P1.4        |
//       --------------------------                                           -------------------
//
//  This example is based on the RobG's example : http://forum.43oh.com/topic/1312-nokia-5110-display
//
//***************************************************************************************

#include <msp430g2553.h>
#include "PCD8544.h"
#include "USCI_A0_uart.h"
#include "timer.h"
#include "adc.h"
#include "printf.h"

//#define DEBUG 1

// KP=3 too weak
#define KP 3
// KI IS !!!!NEGATIVE!!!!
#define KI 10
#define KD 2

#define MAX_CAPTURE 950
#define MIN_CAPTURE 50
// 0x01230123 0x00040000
// there's no need to wind-up more than 1/2 of PWM either way
//#define INTEGRAL_MAX 262144
#define INTEGRAL_MAX 65536
//#define INTEGRAL_MAX 16384
//#define INTEGRAL_MAX 4096

/* */

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


extern bool vco_tracked;
extern bool ref_tracked;

//extern uint16_t dec_pd,
//                  inc_pd,
//                  race_cond,
//                  missed_ref_rising,
//                  missed_ref_falling,
//                  missed_vco_rising,
//                  missed_vco_falling;

int16_t error_current, error_previous, error_max, error_min, error_delta;
//extern volatile uint16_t target_phase_diff;
int16_t p_factor;
int32_t i_factor;
int16_t d_factor;
uint16_t new_pwm;
uint8_t kp = KP, ki = KI, kd = KD;
uint32_t event_alarm, fix_alarm;
#ifdef DEBUG
int16_t slope = 1;
#endif /* DEBUG */

uint16_t pid_loop_count = 0;
uint16_t correction_period = 1, correction_timer = 0, correction_margin = 32;

typedef enum {
  STARTUP = 0,
  WARM_UP = 1,
  LOCKING = 2,
  TRACKING = 3
#ifdef DEBUG
  ,TESTING = 4,
  FORCED = 5,
  SLOPE = 6
#endif /* DEBUG */
} ctrl_state_t;

const char ctrl_state_name[] = { 's', 'w', 'l', 't'
#ifdef DEBUG
  , 'd', 'F', '/'
#endif /* DEBUG */
};
//                   A 5 52.6 6FDD 3.388 -83.4 -232 13 123 -232 -128 -65536 0
//                   0000000000111111111122222222223333333333444444444455555555556666666666
//                   0123456789012345678901234567890123456789012345678901234567890123456789
char monitoring[] = "0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF\r\n";
char *p = &monitoring[0];

ctrl_state_t controller(ctrl_state_t current_state) {
  ctrl_state_t next_state = current_state;
  int16_t curr_phase_diff;
  switch(current_state) {
    case STARTUP:
      next_state = WARM_UP;
      event_alarm = getSeconds();
    break;
    case WARM_UP:
      if((getOCXOTemperature() > ((fix_status == 'A')?(50 << 8):(52 << 8))))
      {
        clearBank(1); clearBank(2); clearBank(3); clearBank(4);
        setAddr(0, 2);  //0123456789ABCD
        writeStringToLCD("waiting");
        setAddr(7*6, 3);
        writeStringToLCD("for fix");
#ifdef DEBUG
        next_state = TESTING;
        event_alarm = getSeconds() + 15;
        setPWM(0x0000);
#else
        next_state = LOCKING;
        fix_alarm = getSeconds() + 2;
#endif
        setTargetPhaseDiff(0);
      }
    break;
    case LOCKING:
      curr_phase_diff = getPhaseDiff();
      if(//(curr_phase_diff > -MAX_CAPTURE) && (curr_phase_diff < MAX_CAPTURE) &&
         //(abs(abs(curr_phase_diff) - 500) < 32) &&
         (fix_status == 'A')
         && ref_tracked
         && vco_tracked
         && (getSeconds() > fix_alarm)
       )
      {
#if 0
        if(curr_phase_diff > 0)
          setTargetPhaseDiff(500);
        else
          setTargetPhaseDiff(-500);
#else
        if((curr_phase_diff > -MIN_CAPTURE) && (curr_phase_diff < MIN_CAPTURE)) {
          if(curr_phase_diff > 0)
            setTargetPhaseDiff(MIN_CAPTURE);
          else
            setTargetPhaseDiff(-MIN_CAPTURE);
        } else {
          if(curr_phase_diff < -MAX_CAPTURE)
            setTargetPhaseDiff(-MAX_CAPTURE);
          else {
            if(curr_phase_diff > MAX_CAPTURE)
              setTargetPhaseDiff(MAX_CAPTURE);
            else
              setTargetPhaseDiff(curr_phase_diff);
            }
        }
#endif
        kp = KP;
        ki = KI;
        kd = KD;
        // target phaes diff should be <-999 ... +999
        while(txBusy());
        char *p = &monitoring[0];
        strprintf(&p, "PID: %i/%i/%i; start tracking %c%i -> %c%i... \r\n", kp, ki, kd,
        curr_phase_diff>0?'+':' ', curr_phase_diff,
        getTargetPhaseDiff()>0?'+':' ', getTargetPhaseDiff());
        *p = 0;
        putstring(monitoring);
        while(txBusy());
        next_state = TRACKING;
        error_max = INT16_MIN;
        error_min = INT16_MAX;
        p = &monitoring[0];
        //             "%c %i   T    PWM PD   F.f  E  driftrate eD DOP P i Il D\r\n",
        //             "%c%i    T    PWM PD   F.f  diff   E     eD    DOP P  i Il D\r\n",
        strprintf(&p, "fix sats temp pwm ctrl freq phdiff error delta dop P I I D\r\n");
        *p = 0;
        putstring(monitoring);
        while(txBusy());
      }
    break;
    case TRACKING:
      // track phase
      // POSITIVE error - we need to chase Ref
      // NEGATIVE error - we need to slow down and let REF catch up
      error_current = getPhaseDiff()- getTargetPhaseDiff();

      if(error_current > error_max)
        error_max = error_current;
      if(error_current < error_min)
        error_min = error_current;

      // +500 is angle of 90 degrees difference between REF and VCO
      // phase_diff <-1000 ... +1000>
      p_factor = error_current;
      i_factor += error_current;
      if(i_factor > (int32_t)(INTEGRAL_MAX))
        i_factor = (int32_t)(INTEGRAL_MAX);
      if(i_factor < (int32_t)(-INTEGRAL_MAX))
        i_factor = (int32_t)(-INTEGRAL_MAX);
      d_factor = (error_current - error_previous);
      error_previous = error_current;
      // p_factor <-500 .. + 1500>
      new_pwm = TA0CCR1_DEF
                + (p_factor << kp)
                + (i_factor >> ki);
                + (d_factor << kd);

      setPWM(new_pwm);

      if(fix_status != 'A') {
        next_state = WARM_UP;
        event_alarm = getSeconds();
        ref_tracked = FALSE;
        vco_tracked = FALSE;
      }
    break;
#ifdef DEBUG
    case TESTING:
      if(getSeconds() == event_alarm) {
        //next_state = FORCED;
        next_state = SLOPE;
        event_alarm = getSeconds() + 30;
      }
    break;
    case FORCED:
      if(getSeconds() == event_alarm) {
        if(getPWM() > 0x8000) {
          setPWM(0x0000);
        } else {
          setPWM(0xFFFF);
        }
        event_alarm = getSeconds() + 60;
      }
    break;
    case SLOPE:
      setPWM(getPWM() + slope);
      if(slope > 0) {
        if(getPWM() > (0xFF00 - slope)) {
          //next_state = TESTING;
          slope = -slope;
        }
      } else {
        if(getPWM() < (0x00FF - slope)) {
          slope = (abs(slope) + 1);
          next_state = TESTING;
          event_alarm = getSeconds() + 10;
        }
      }
    break;
#endif
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
  setAddr(0, 5);
  writeStringToLCD("(c)2024 sq7bti");
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
  uint8_t x = 0, y = 0, y_min = 255, y_max = 0, y_scale = 4;

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
#ifdef OVERCLOCK
          phase_difference(5, (getPhaseDiff() + 1003) / 24, ((getTargetPhaseDiff()==INT16_MAX)?-1:(getTargetPhaseDiff() + 1003) / 24));
#else
          phase_difference(5, (getPhaseDiff() + 800) / 24, ((getTargetPhaseDiff()==INT16_MAX)?-1:(getTargetPhaseDiff() + 800) / 24));
#endif /* OVERCLOCK */
          //setAddr(0,1);
          //writeDecToLCD(race_cond);
          //setAddr(0,2);
          //writeDecToLCD(missed_ref_rising);
          //setAddr(0,3);
          //writeDecToLCD(missed_ref_falling);
          //setAddr(42,2);
          //writeDecToLCD(missed_vco_rising);
          //setAddr(42,3);
          //writeDecToLCD(missed_vco_falling);
        break;
#ifdef DEBUG
        case TESTING:
        case FORCED:
        case SLOPE:
#endif /* DEBUG */
        case TRACKING:
          if(abs(error_current) > 16)
          {
#ifdef OVERCLOCK
            phase_difference(5, (getPhaseDiff() + 1003) / 24, (getTargetPhaseDiff() + 1003) / 24);
#else
            phase_difference(5, (getPhaseDiff() + 800) / 24, (getTargetPhaseDiff() + 800) / 24);
#endif /* OVERCLOCK */
          } else {
            setAddr(0, 5);
            writeMHzToLCD(getOCXO());
            if(y_scale)
              writeDecToLCD(y_scale);
          }
          //setAddr(0, (error_current > -5)?4:1);
          setAddr(0, 1);
          writeWordToLCD(getPWM());
          //setAddr(84 - 6*6, (error_current > -5)?4:1);
          setAddr(84 - 6*5, 1);
          writeQ4CToLCD(getCtrl());
          //writeCharToLCD('V');

          //=================
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
      uint32_t freq_meas = getOCXO() / 5;
#ifdef USE_10MHZ_INPUT_AS_TACLK
  #if CAPTURE_MULT <= 10000
      int16_t freq_off_hz = getOCXO() - 10000000UL;
      uint16_t freq_off_tenth = 0;
  #endif
  #if CAPTURE_MULT == 50000
      int16_t freq_off_hz = ((int16_t)(getOCXO() - 50000000UL))/5;
      uint16_t freq_off_tenth = abs(((int16_t)(getOCXO() - 50000000UL)))%5;
      //freq_off_tenth &= 0x00FF;
      freq_off_tenth *= 2;
      //freq_off_tenth >>= 8;
      //freq_off_tenth /= 5;
  #endif
#else
      int16_t freq_off_hz = getOCXO();
      uint16_t freq_off_tenth = 0;
#endif
      //strprintf(&p, "PWM %x ph %i P:%i I:%i D:%i\r\n", getPWM(), phase_diff, p_factor, i_factor, d_factor);
      //strprintf(&p, "%c%i T%q P%x %i E:%i P:%i I:%l D:%i\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getPWM(),
      //        getOCXO() - 10000000, error_current,
      //        p_factor, i_factor, d_factor);
      strprintf(&p,
#if CAPTURE_MULT <= 10000
                    "%c %i %q %x %r %i %i %i %i %i %i %i %l %l %i\r\n",
#endif
#if CAPTURE_MULT == 50000
                    //"%c%i T PWM PD F.f  ref vco pe ped E eD P  i Il D\r\n",
                    //"%c %i %q %x %r %i.%i %i %i %i %i %i %i %l %l %i\r\n",
                    //"%c%i T PWM PD F.f diff E eD DOP P  i Il D\r\n",
                    "%c %i %q %x %r %i.%i %i %i %i %q %i %i %l %i\r\n",
                    //"%n %x : (%c) -> (%c) %i\r\n",
#endif
                    //pllcount, pllstate, i2h(pllstate >> 4), i2h(pllstate & 0x0F), getPhaseDiff());
                    fix_status, used_sats, getOCXOTemperature(), getPWM(), getCtrl(),
                    freq_off_hz, freq_off_tenth,
                    getPhaseDiff(),
                    error_current, error_delta,
                    ((uint16_t)hdop << 8) / 100,
                    p_factor, (int16_t)(i_factor >> KI), i_factor, d_factor);
      //strprintf(&p, "F%c%i T%q POT %x = %rV %l\r\n",
      //        fix_status, used_sats, getOCXOTemperature(), getCtrl(), getCtrl(), getOCXO());
      //strprintf(&p, "Int %q Oven %q ADC %r\r\n",
      //            getIntTemperature(), getOCXOTemperature(), getCtrl());
      *p = 0;
      putstring(monitoring);

      error_delta = error_max - error_min;
      error_max = INT16_MIN;
      error_min = INT16_MAX;

      clearTrigFlag(TRIGGER_LOG);
    }

    // executed once a second (time!)
    if(getTrigFlag(TRIGGER_SEC)) {
      switch(gpsdo_ctrl_state) {
        case WARM_UP:
          /*/=============
          y = 24 - (getPhaseDiff() >> 6);
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0);
          graph(x, y, 24);
          //  pixel(x,24);
          ++x;
          if(x > 84)
            x = 0;
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0xFF);
          //=========*/
        break;
        case LOCKING:
          // "waiting"
          setAddr(6*8, 2);
          writeDecToLCD(getSeconds() - event_alarm);
          writeCharToLCD('s');
        break;
        case TRACKING:
          //=============
          y = 24 - (error_current >> y_scale); ///1); //(int8_t)((getPWM() - TA0CCR1_DEF) >> 3);
          if(y > y_max)
            y_max = y;
          if(y < y_min)
            y_min = y;
          //y = 40 - (getPWM() >> 11);
          setAddr(x, 1);
          writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 4);
          writeToLCD(LCD5110_DATA, 0);
          if(x == 0) {
            // 5 = +-512
            // 4 = +-256
            // 3 = +-128
            // 2 = +-64
            // 1 = +-32
            // 0 = +-16
            // y_max = 24 - E -> E = 24 - y_max
            setAddr(84 - 6, 5);
            writeDecToLCD(y_scale);
          }

          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0);

          graph(x, y, 24);
          //  pixel(x,24);
          ++x;
          if(x > 84) {
            // 8 .. 15
            // 16 .. 23
            // 24 .. 31
            // 32 .. 39
            if((y_scale != 0) && (y_max < 32) && (y_min > 16))
              --y_scale;
            if((y_scale != 0) && (y_max < 28) && (y_min > 20))
              --y_scale;
            if((y_scale < 5) && ((y_max > 39) || (y_min < 8)))
              ++y_scale;
            y_min = 40;
            y_max = 8;
            x = 0;
          }
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0xFF);
          //=========
        }
        if(error_delta > 5) {
          setAddr(84 - 4*6, 4);
          writeStringToLCD(hdop_str);
        //} else {
        //  if(error_delta < 3) {
        //    setAddr(84 - 4*6, 4);
        //    writeStringToLCD("    ");
        //  }
        }
      clearTrigFlag(TRIGGER_SEC);
    }

    // execute reeaally slow (like warming up)
    if(getTrigFlag(TRIGGER_SLW)) {
      switch(gpsdo_ctrl_state) {
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
          //pixel(x,y);
          // 50deg as dotted line
          graph(x, y, 10);
          ++x;
          if(x > 84)
            x = 0;
          setAddr(x, 1); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 2); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 3); writeToLCD(LCD5110_DATA, 0xFF);
          setAddr(x, 4); writeToLCD(LCD5110_DATA, 0xFF);
        //====================
        break;
        default:
        break;
      }

      clearTrigFlag(TRIGGER_SLW);
    }

    // executed to follow ADC measurements
    if(getTrigFlag(TRIGGER_ADC)) {
      setAddr(0, 0);
      clearBank(0);
      setInverse(getOCXOTemperature() < (50<<8));
      //writeQ88ToLCD(getOCXOTemperature());
      writeDecToLCD((getOCXOTemperature() >> 8) + ((getOCXOTemperature() & 0x0080) >> 7));
      writeCharToLCD(0x7F);
      writeCharToLCD('C');
      setInverse(FALSE);
      writeDecToLCD((getIntTemperature() >> 8) + ((getIntTemperature() & 0x0080) >> 7));
      writeCharToLCD(0x7F);
      writeCharToLCD('C');
      //writeCharToLCD(' ');
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

      // error abs()<8 means the phase error is less than 4 cycles of 10MHz
      // in normal circumstances it would mean the frequency offset is
      // less than 4Hz from the reference, but in conjunction with delta (variation)
      // less than 2 the VCO signal should be of acceptable stability
      if(gpsdo_ctrl_state == TRACKING)
        setInverse((abs(error_current) < 8) && (error_delta < 2));
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

          //writeQ4CToLCD(getCtrl());
          //writeCharToLCD('V');

          //writeWordToLCD(phase_comp_raw_value);
          //writeStringToLCD(" = 0x");
          //writeIntToLCD(getPhaseDiff());
          //writeCharToLCD(' ');

          //writeIntToLCD(getTargetPhaseDiff());
          //writeCharToLCD(' ');

          //writeQ4CToLCD(getCtrl());
          //writeCharToLCD('V');
          //writeCharToLCD(' ');
          //writeWordToLCD(getPWM());

          //writeIntToLCD(2 * phase_driftrate);
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
          //strprintf(&t, "%rV %q%cC", getCtrl(), getIntTemperature(), 0x7F);
          //*t = 0;
          //writeStringToLCD(sbuff);
#else
          //writeQ4CToLCD(getCtrl());
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

    // execute PID controller
    if(getTrigFlag(TRIGGER_PID)) {
      gpsdo_ctrl_state = controller(gpsdo_ctrl_state);
      clearTrigFlag(TRIGGER_PID);
    }

    if(!getTrigFlag(TRIGGER_ANY))
      LPM0;
  } // eof while()
} // eof main
