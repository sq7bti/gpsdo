#include "timer.h"

#define USE_10MHZ_INPUT_AS_TACLK 1
#define CAPTURE_MULT 10000

static volatile uint32_t millis = 0, seconds = 0;
volatile uint32_t ocxo_count = 0;
volatile uint32_t capture_count_acc = 0;
volatile uint16_t capture_count = 0;
volatile uint16_t capture_mult = CAPTURE_MULT;
static volatile uint16_t capture_overflow = 0;

volatile uint16_t previous_rising_ref = 0;
volatile uint16_t current_rising_ref = 0;
volatile uint16_t period_ref = 0;
volatile uint16_t previous_rising_vco = 0;
volatile uint16_t current_rising_vco = 0;
volatile uint16_t period_vco = 0;

// positive means VCO is ahead of GPS signal
// negative means VCO is behind GPS signal
volatile int16_t phase_diff = 0;
volatile int16_t phase_driftrate = 0;
volatile bool vco_tracked = FALSE;
volatile bool ref_tracked = FALSE;

uint32_t getMillis() { return millis/10; };
uint32_t getSeconds() { return seconds; };
uint32_t getOCXO() { return ocxo_count; };
uint32_t getCAP() { return ((uint32_t)capture_overflow << 16) + capture_count; };

extern uint16_t* adc_values;
extern uint16_t* int_temp_sensor_values;

extern volatile char fix_status;

void initTIMER(void)
{
  // 10kHz comparator P2.1 and P2.4
  P2SEL |= BIT1 | BIT4;                       // Use P2.1 and P2.1 as Timer1_A0 inputs
	P2SEL2 &= ~(BIT1 | BIT4);                   //
	P2DIR &= ~(BIT1 | BIT4);                    // inputs
	P2OUT &= ~(BIT1 | BIT4);                    //
	P2REN |= BIT1 | BIT4;                       // Enable pull down resistor to reduce stray counts

#ifdef USE_10MHZ_INPUT_AS_TACLK
  // 10MHz input from OCXO P1.0
  P1SEL |= BIT0;                              // Use P1.0 as Timer1_A0 TACLK
	P1SEL2 &= ~BIT0;                            //
	P1DIR &= ~BIT0;                             // inputs
	P1OUT &= ~BIT0;                             //
	P1REN |= BIT0;                              // Enable pull down resistor to reduce stray counts
#endif /* USE_10MHZ_INPUT_AS_TACLK */

  // pwm ouput on P1.6
  P1SEL |= BIT6;                              // Use P1.6 as TimerA1 output
	P1SEL2 &= ~BIT6;                            // Timer1_A3.TAO
	P1DIR |= BIT6;                              // output

  // capture mode; rising edge; input select CCIxA; enable interrupts
  TA0CCTL0 = CAP | CM_1 | CCIS_1 | CCIE;
  // capture mode; falling edge; input select CCIxA; enable interrupts
  //TA0CCTL0 = CAP | CM_2 | CCIS_1 | CCIE;

  // CCR1 - PWM output on P1.6
  TA0CCTL1 = OUTMOD_3; // set/reset output - inverted with 3V3/5V shifter
  TA0CCR1 = TA0CCR1_DEF;   // 50% PWM

#ifdef USE_10MHZ_INPUT_AS_TACLK
  // TACLK - connected 10MHz from OCXO; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_0 | MC_2 | TACLR;
#else
  // SMCLK - 16MHz; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_2 | MC_2 | TACLR;
#endif /* USE_10MHZ_INPUT_AS_TACLK */


  // Timer1_A
  // CCR1 - Capture input on P2.1 - 10kHz reference signal from GPS module
  TA1CCTL1 = CAP | CM_1 | CCIE | SCS | CCIS_0; // | CCIE; // capture on rising edge of CCI1A

  // CCR2 - Capture input on P2.4)
  TA1CCTL2 = CAP | CM_1 | CCIE | SCS | CCIS_0; // | CCIE; // capture on rising edge of CCI1A

  // timer 1 used to track 10kHz from reference (GPS) and VCO (OCXO)
  TA1CTL = TASSEL_2 | MC_2 | TACLR; // SMCLK; continuous
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_iSR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_iSR (void)
#else
#error Compiler not supported!
#endif
{
//  if(TA0CCTL0 & CCIFG) {
#if CAPTURE_MULT == 1
    ocxo_count = TACCR0 - capture_count;
    capture_count = TACCR0;
#else
  #if CAPTURE_MULT < 7
    if(!capture_mult) {
      ocxo_count = TACCR0 - capture_count;
      capture_count = TACCR0;
      capture_mult = CAPTURE_MULT;
    }
    --capture_mult;
  #else
    if(!capture_mult) {
      capture_mult = CAPTURE_MULT;
      //ocxo_count = capture_count_acc - CAPTURE_MULT * 10000UL;
      ocxo_count = capture_count_acc;
      capture_count_acc = 0;
    }
    capture_count_acc += TACCR0 - capture_count;
    capture_count = TACCR0;
    --capture_mult;
  #endif

#endif
//  }
}

volatile int16_t new_phase_diff;

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_iSR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) Timer1_A1_iSR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV, 0x0A)) {
    case TA1IV_NONE: // 0x00
      break;
    case TA1IV_TACCR1: // 0x02
      // rising edge of reference signal from GPS
      current_rising_ref = TA1CCR1;
      period_ref = current_rising_ref - previous_rising_ref;
      previous_rising_ref = current_rising_ref;
      // it means ref is delayed, vco was just captured
      if((TA1CCTL2 & CCI) && !(TA1IV & TA1IV_TACCR2)) {
      //if(TA1CCTL2 & CCI) {
        //new_phase_diff = current_rising_ref - (TA1IV & TA1IV_TACCR2)?TA1CCR2:current_rising_vco;
        new_phase_diff = current_rising_ref - current_rising_vco;
        phase_driftrate = new_phase_diff - phase_diff;
        phase_diff = new_phase_diff;
      }
      ref_tracked = TRUE;
      TA1CCTL1 &= ~CCIFG;
      break;
    case TA1IV_TACCR2: // 0x04
      // rising edge of controlled signal from VCO
      current_rising_vco = TA1CCR2;
      period_vco = current_rising_vco - previous_rising_vco;
      previous_rising_vco = current_rising_vco;
      // it means vco is delayed, ref was just captured
      if((TA1CCTL1 & CCI) && !(TA1IV & TA1IV_TACCR1)) {
      //if(TA1CCTL1 & CCI) {
        //new_phase_diff = (TA1IV & TA1IV_TACCR1)?TA1CCR1:current_rising_ref - current_rising_vco;
        new_phase_diff = current_rising_ref - current_rising_vco;
        phase_driftrate = new_phase_diff - phase_diff;
        phase_diff = new_phase_diff;
      }
      vco_tracked = TRUE;
      TA1CCTL2 &= ~CCIFG;
      break;
    case TA1IV_6: // reserved 0x06
      break;
    case TA1IV_8: // reserved 0x08
      break;
    case TA1IV_TAIFG: // 0x0A
      break;
    default:
      break;
  }
  //LPM0_EXIT;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer_ISR (void)
#else
#error Compiler not supported!
#endif
{
  capture_overflow = 0;
  // with every fire of WDTimer it is :
  // 10kHz/64
  millis += 64;
  //millis += 512;
  if(millis > 10000) {
    millis -= 10000;
    ++seconds;
  }

  if(!(ADC10CTL1 & ADC10BUSY)) {
      ADC10CTL0 |= ENC | ADC10ON | ADC10SC;             // Sampling and conversion start
  }
}
