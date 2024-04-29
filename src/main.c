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
//      |           10MHz |<-- VCO output --------------------------|2      P1.0        |
//      |           10kHz |<-- VCO output divided % 1000 -----------|12     P2.4        |
//      |            Vref |<-- VCO input ---------------------o<|---|14     P1.6 (neg)  |
//      |            LM35 |<-- temperature -------------------------|5      P1.3        |
//       -----------------                                          |                   |
//                                                                  |                   |
//              4046                                                |                   |
//       -----------------                                          |                   |
//      |              PC |<-- Phase comparator output -------------|6      P1.4        |
//       -----------------                                          |                   |
//                                                                  |                   |
//                                                                  |                   |
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

#define NMEA_RMC_LOCK 23
#define NMEA_RMC_TIM 6
#define NMEA_RMC_LAT 25
#define NMEA_RMC_LNG 27

#define KP +7
#define KI -6
#define KD 0

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

uint16_t ocxo_temperature, adc_val;

/* WDT is clocked by fACLK (assumed 10kHz) */
#define WDT_XDLY_3267       (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL)                 /* 3267.8ms  " /32768 */
#define WDT_XDLY_819        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS0)          /* 819.2ms   " /8192  */
#define WDT_XDLY_51         (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1)          /* 51.2ms    " /512   */
#define WDT_XDLY_6_4        (WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1+WDTIS0)   /* 6.4ms   " /64    */

void initCPU(void){
	  //WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    WDTCTL = WDT_XDLY_6_4;
    //WDTCTL = WDT_XDLY_51;
    //WDTCTL = WDT_MDLY_0_5;                    //equivalent to half millisecond
    IE1 |= WDTIE;                             // Enable WDT interrupt}

	  DCOCTL = 0;                               // Run at 16 MHz
	  BCSCTL1 = CALBC1_16MHZ;                   //
	  DCOCTL  = CALDCO_16MHZ;                   //
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

extern uint16_t* ocxo_temp_raw_values;
extern uint16_t ocxo_temp_raw_value;
extern uint16_t phase_comp_raw_value;
extern uint16_t* int_temp_sensor_values;
extern uint16_t int_temp_sensor_value;
extern uint16_t frame_counter[];
extern uint16_t bad_crc_counter, chars_count;

extern uint16_t period_ref;
extern uint16_t period_vco;

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

int main(void) {

    initCPU();
    initUART();
    initSPI();
    initTIMER();
    initADC();

    __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled

    int c, i, k = 0;

    for(i = 16; i > 0; --i)
      __delay_cycles(500000);

    initLCD();
    clearLCD();

    for(i = 16; i > 0; --i)
      __delay_cycles(500000);

    setAddr(0, 3);
    if(IFG1 & OFIFG)
      writeStringToLCD("Xtal fault");

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
    clearBank(1);
    clearBank(2);
    clearBank(3);
    while(1) {
        //clearLCD();
        //setAddr(0, 0);

      //setAddr(0, 2);
      //clearBank(2);
      //if(phase_diff != 0)
      //  writeCharToLCD(phase_diff > 0?'+':'-');
      //else
      //  writeCharToLCD(' ');
      //writeDecToLCD(abs(phase_diff));
      //if(phase_driftrate != 0)
      //  writeCharToLCD(phase_driftrate > 0?'+':'-');
      //else
      //  writeCharToLCD(' ');
      //writeDecToLCD(abs(phase_driftrate));

      //writeCharToLCD(' ');
      //writeDecToLCD(period_ref);
      //writeCharToLCD(' ');
      //writeDecToLCD(period_vco);
      phase_difference(3, (phase_diff + 800) / 20);

      if(!getticks()) {
        setAddr(84 - 6, 5);
        switch(msg_count % 4) {
          case 0: writeCharToLCD('-'); break;
          case 1: writeCharToLCD('/'); break;
          case 2: writeCharToLCD('|'); break;
          case 3: writeCharToLCD('\\'); break;
        }

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
        //writeWordToLCD(phase_comp_raw_value);
        //writeStringToLCD(" = 0x");
        if(phase_diff != 0)
          writeCharToLCD(phase_diff > 0?'+':'-');
        else
          writeCharToLCD(' ');
        writeDecToLCD(abs(phase_diff));
        writeCharToLCD(' ');
        if(target_phase_diff != 0)
          writeCharToLCD(target_phase_diff > 0?'+':'-');
        else
          writeCharToLCD(' ');
        writeDecToLCD(abs(target_phase_diff));
        writeCharToLCD(' ');
        writeWordToLCD(TA0CCR1);


        setAddr(0, 2);
        clearBank(2);

        if(fix_status != 'A') {
          if(phase_driftrate != 0)
            writeCharToLCD(phase_driftrate > 0?'+':'-');
          else
            writeCharToLCD(' ');
          writeDecToLCD(abs(phase_driftrate));
        } else {
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
        }

        //setInverse(getPhaseDet() > 1<<12);
        //writeQ4CToLCD(getPhaseDet());
        //setInverse(FALSE);
        //writeCharToLCD('V');
        //bargraph(2, phase_comp_raw_value/194);


/*        setAddr(0,2);
        clearBank(2);
        if(phase_diff < 0) {
          writeCharToLCD('-');
          writeDecToLCD(-phase_diff);
        } else {
          writeCharToLCD('+');
          writeDecToLCD((uint16_t)phase_diff);
        }
        */

        if((fix_status == 'A') && (getOCXOTemperature() > (50 << 8))) {
          if(target_phase_diff == 32000) {
            if(ref_tracked && vco_tracked) {
              target_phase_diff = phase_diff;
              if(target_phase_diff < -400)
                target_phase_diff = -400;
              if(target_phase_diff > 400)
                  target_phase_diff = 400;
              if((target_phase_diff > -8) && (target_phase_diff < 8))
                target_phase_diff = 8;
            }
          } else {
            if((phase_diff < 800) && (phase_diff > -800)) {
              // track phase
              error_current = target_phase_diff - phase_diff;
              // +400 is angle of 90 degrees difference between REF and VCO
              // phase_diff <-800 ... +800>
              p_factor = error_current;
              i_factor += error_current;
              d_factor = (error_previous - error_current);

              // p_factor <-400 .. + 1200>
              TA0CCR1 = TA0CCR1_DEF
                      + (p_factor << KP)
                      + (i_factor << KI)
                      + (d_factor << KD);

              error_previous = error_current;
            }
          }
        }

      }

      if(new_frame) { //} && crc_good) {
        new_frame = FALSE;

/*        setAddr(0, 0);
        clearBank(0);
        writeStringToLCD("UNK");
        writeDecToLCD(frame_counter[0]);
        writeStringToLCD("RMC");
        writeDecToLCD(frame_counter[1]);
        setAddr(0, 1);
        clearBank(1);
        writeStringToLCD("VTG");
        writeDecToLCD(frame_counter[2]);
        writeStringToLCD("GGA");
        writeDecToLCD(frame_counter[3]);
        setAddr(0, 2);
        clearBank(2);
        writeStringToLCD("GSA");
        writeDecToLCD(frame_counter[4]);
        writeStringToLCD("GSV");
        writeDecToLCD(frame_counter[5]);

        setAddr(0, 3);
        clearBank(3);
        writeStringToLCD("BAD");
        writeDecToLCD(bad_crc_counter);*/

        //setAddr(54, 4);
        //writeDecToLCD(getSeconds());
        //writeCharToLCD('s');
        //writeDecToLCD(chars_count);

//        clearBank(1);
//        setAddr(0, 1);
//        writeDecToLCD(getOCXO());

/*        clearBank(2);
        setAddr(0, 2);
        writeHexToLCD(ocxo_temp_raw_value);
        writeHexToLCD(ocxo_temp_raw_values[0]);
        writeHexToLCD(ocxo_temp_raw_values[1]);
        clearBank(3);
        setAddr(0, 3);
        writeHexToLCD(ocxo_temp_raw_values[2]);
        writeHexToLCD(ocxo_temp_raw_values[3]);
        writeHexToLCD(ocxo_temp_raw_values[4]);
        //writeDecToLCD(getCAP());*/

        //pixel(k, 8 * 4 - (ocxo_temperature >> 9));
        //++k;
        //k %= 84;

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
/*            for(c = 0; c < 12; ++c) {
              if((latitude_upd & (1<<c)) && (lat_array[c] >= 0)) {
                setAddr(lat_array[c]*6, 2);
                writeCharToLCD(latitude[c]);
                latitude_upd &= ~(1<<c);
              }
            }
            for(c = 0; c < 13; ++c) {
              if((longitude_upd & (1<<c)) && (lon_array[c]) >= 0) {
                setAddr(lon_array[c]*6, 3);
                writeCharToLCD(longitude[c]);
                longitude_upd &= ~(1<<c);
              }
            }*/
            if(fix_status_upd) {
              setAddr(84 - (6 * 4), 5);
              writeCharToLCD(fix_status);
              fix_status_upd = FALSE;
            }
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
        if(frame_type == GGA) {
          for(i; i; --i)
            clearBank(i);
          i = 0;
        }
        switch(frame_type) {
          case RMC:
          case GGA:
          case GSA:
          case GSV:
          case VTG:
          case UNKNOWN:
//              lock_status = rxbuffer[NMEA_RMC_LOCK];
//              for(c = NMEA_RMC_TIM; c < NMEA_RMC_TIM + 6; c++)
//                writeCharToLCD(rxbuffer[c]);
            break;
          default:
            clearBank(i);
            setAddr(0, i);
            for(c = 2; c < 8; c++)
              writeCharToLCD(rxbuffer[c]);
            setAddr(84 - (5*6), i);
            writeByteToLCD(checksum);
            writeCharToLCD(crc_good?'=':'x');
            writeCharToLCD(rxbuffer[checksum_idx-1]);
            writeCharToLCD(rxbuffer[checksum_idx]);

            ++i;
            if(i>3)
              i = 0;
            break;
          }
          crc_good = FALSE;
        }
        LPM0;
    } // eof while()
} // eof main
