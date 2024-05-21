#include "timer.h"

#define PHASE_DIFF_AVG 9
//#define PERIOD_AVG 10
#define PWM_MARGIN 255

volatile uint8_t trigger_flags = 0;
static volatile uint32_t millis = 0, seconds = 0;
volatile uint32_t ocxo_count = 0;
volatile uint32_t capture_count_acc = 0;
volatile uint16_t capture_count = 0;
volatile uint16_t capture_mult = CAPTURE_MULT;
static volatile uint16_t pwm_output_duty, pwm_output_low;

//volatile uint16_t previous_rising_ref = 0;
volatile uint16_t current_rising_ref = 0;
//volatile uint16_t previous_falling_ref = 0;
volatile uint16_t current_falling_ref = 0;
//volatile uint32_t period_ref = 0;
//volatile uint16_t previous_rising_vco = 0;
volatile uint16_t current_rising_vco = 0;
//volatile uint16_t previous_falling_vco = 0;
volatile uint16_t current_falling_vco = 0;
//volatile uint32_t period_vco = 0;

// positive means VCO is ahead of GPS signal
// negative means VCO is behind GPS signal
volatile int32_t phase_diff = INT32_MAX;
volatile int16_t target_phase_diff = INT16_MAX;
volatile int16_t phase_diff_prev = 0, old_phase_diff;
volatile int16_t raw_phase_diff;
volatile bool vco_tracked = FALSE;
volatile bool ref_tracked = FALSE;

uint32_t getMillis() { return millis/10; };
uint32_t getSeconds() { return seconds; };
uint32_t getOCXO() { return ocxo_count; };
uint16_t getPWM(void) { return pwm_output_duty; };
bool getTrigFlag(int8_t id) {
  if(id == TRIGGER_ANY)
    return trigger_flags != 0;
  return trigger_flags & (0x01 << id);
};
void setTrigFlag(uint8_t id) { trigger_flags |= (0x01 << id); };
void clearTrigFlag(uint8_t id) { trigger_flags &= ~(0x01 << id); };

void setPWM(uint16_t p) {
  pwm_output_duty = p;
  if(p < PWM_MARGIN)
    pwm_output_duty = PWM_MARGIN;
  if(p > (0xFFFF - PWM_MARGIN))
    pwm_output_duty = (0xFFFF - PWM_MARGIN);
};

int16_t getPhaseDiff(void) { return (int16_t)(phase_diff >> PHASE_DIFF_AVG); };
int16_t getTargetPhaseDiff() { return target_phase_diff; }; // >> PHASE_DIFF_AVG; };
void setTargetPhaseDiff(int16_t p) { target_phase_diff = p; }; // << PHASE_DIFF_AVG; };

//uint16_t getPeriodRef() { return (uint16_t)(period_ref >> PERIOD_AVG); };
//uint16_t getPeriodVCO() { return (uint16_t)(period_vco >> PERIOD_AVG); };

extern uint16_t* adc_values;
extern uint16_t* int_temp_sensor_values;

extern volatile char fix_status;

extern volatile bool rx_busy;

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
  //P1SEL |= BIT6;                              // Use P1.6 as TimerA1 output
	//P1SEL2 &= ~BIT6;                            // Timer1_A3.TAO
	//P1DIR |= BIT6;                              // output

  // if capture mode is on on CCR0 - CCR1 output is switched off
  // capture mode; rising edge; input select CCIxA; enable interrupts
  //TA0CCTL0 = CAP | CM_1 | CCIS_1 | CCIE;
  // capture mode; falling edge; input select CCIxA; enable interrupts
  TA0CCTL0 = CAP | CM_2 | CCIS_1 | CCIE;

  // CCR1 - PWM output on P1.6
  //TA0CCTL1 = OUTMOD_7; // set/reset output - inverted with 3V3/5V shifter
  //TA0CCTL1 = OUTMOD_7; // set/reset output - inverted with 3V3/5V shifter
  //TA0CCTL1 = OUTMOD_1 | CCIE; // set output - will be toggled to OUTMOD_5 (reset) in ISR
  //TA0CCTL1 = OUTMOD_4 | CCIE; // set output - will be toggled to OUTMOD_5 (reset) in ISR
  //TA0CCR1 = pwm_output_duty; //TA0CCR1_DEF;   // 50% PWM

#ifdef USE_10MHZ_INPUT_AS_TACLK
  // TACLK - connected 10MHz from OCXO; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_0 | MC_2 | TACLR;
#else
  // SMCLK - 16MHz; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_2 | MC_2 | TACLR;
#endif /* USE_10MHZ_INPUT_AS_TACLK */


  // Timer1_A
  // CCR0 - generate PWM by ISR
  TA1CCTL0 = CCIE;                                        // interrupt enable
  P2DIR |= BIT0;                                          // P2.0 output
  P2OUT |= BIT0;
  //TA1CCR0 = pwm_output_duty;                              // high state duration
  //setPWM(TA0CCR1_DEF);
  pwm_output_duty = TA0CCR1_DEF;
  //pwm_pol = true;

  //setPWM(0x0000);
  //setPWM(0xFFFF);

  // CCR1 - Capture input on P2.1 - 10kHz reference signal from GPS module
  //     capture   rising irq    synch
  //TA1CCTL1 = CAP | CM_1 | CCIE | SCS | CCIS_0; // capture on rising edge of CCI1A
  TA1CCTL1 = CAP | CM_3 | CCIE | SCS | CCIS_0; // capture on both edges of CCI1A

  // CCR2 - Capture input on P2.4)
  //TA1CCTL2 = CAP | CM_1 | CCIE | SCS | CCIS_0; // capture on rising edge of CCI1A
  TA1CCTL2 = CAP | CM_3 | CCIE | SCS | CCIS_0; // capture on both edges of CCI1A

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
    if(TA0CCTL0 & COV) {
      TA0CCTL0 &= ~COV;
    } else {
      capture_count_acc += TACCR0 - capture_count;
      --capture_mult;
    }
    capture_count = TACCR0;
  #endif

#endif
//  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_iSR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_iSR (void)
#else
#error Compiler not supported!
#endif
{
  // polarity reversed with 74hct04 inverter
  // MSP outputs 3V3, hct04 inverts to 5V logic
  if(P2OUT & BIT0) {
    TA1CCR0 += pwm_output_duty;
    P2OUT &= ~BIT0;
    pwm_output_low = ~pwm_output_duty - 1;
  } else {
    TA1CCR0 += pwm_output_low;
    P2OUT |= BIT0;
  }
  TA0CCTL0 &= ~CCIFG;
}

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
      if(TA1CCTL1 & CCI)
      {
        // rising edge of reference signal from GPS
        current_rising_ref = TA1CCR1;
        // if in the meantime TA1IV_TACCR2 was also triggered wait to calculate it in vco ISR
        //if(!(TA1IV & TA1IV_TACCR2))
        //if((TA1CCTL2 & CCI) && !(TA1IV & TA1IV_TACCR2)) {
        // VCO is already high
        // it means ref is delayed, vco was just captured
        // phase_diff is NEGATIVE
        // if phase_diff rises, it means VCO is faster than REF
        // if phase_diff drops, it means VCO is slower than REF
        //if(TA1CCTL2 & CCI)
        //if(!(TA1IV & TA1IV_TACCR2))
        if((TA1CCTL2 & CCI) && !(TA1IV & TA1IV_TACCR2))
        {
          //raw_phase_diff = (TA1IV & TA1IV_TACCR2)?TA1CCR2:current_rising_vco - current_rising_ref; // - target_phase_diff;
          raw_phase_diff = current_rising_vco - current_rising_ref; // - target_phase_diff;

          //if(abs(raw_phase_diff) < (1000 << PHASE_DIFF_AVG))
          {
            if(phase_diff == INT32_MAX) {
              phase_diff = (int32_t)(raw_phase_diff) << PHASE_DIFF_AVG;
            } else {
              phase_diff -= phase_diff >> PHASE_DIFF_AVG;
              phase_diff += (int32_t)(raw_phase_diff);
              ref_tracked = TRUE;
              vco_tracked = TRUE;
            }
          }
        }
        //previous_rising_ref = current_rising_ref;
      } else { //if(TA1CCTL1 & CCI)
        // falling edge of reference signal from GPS
        current_falling_ref = TA1CCR1;
        // if in the meantime TA1IV_TACCR2 was also triggered wait to calculate it in vco ISR
        //if(!(TA1IV & TA1IV_TACCR2))
        //if((TA1CCTL2 & CCI) && !(TA1IV & TA1IV_TACCR2)) {
        // VCO is already high
        // it means ref is delayed, vco was just captured
        // phase_diff is NEGATIVE
        // if phase_diff rises, it means VCO is faster than REF
        // if phase_diff drops, it means VCO is slower than REF
        //if(TA1CCTL2 & CCI)
        //if(!(TA1IV & TA1IV_TACCR2))
        if((TA1CCTL2 & CCI) && !(TA1IV & TA1IV_TACCR2))
        {
          //raw_phase_diff = (TA1IV & TA1IV_TACCR2)?TA1CCR2:current_rising_vco - current_rising_ref; // - target_phase_diff;
          raw_phase_diff = current_falling_vco - current_falling_ref; // - target_phase_diff;

          //if(abs(raw_phase_diff) < (1000 << PHASE_DIFF_AVG))
          {
            if(phase_diff == INT32_MAX) {
              phase_diff = (int32_t)(raw_phase_diff) << PHASE_DIFF_AVG;
            } else {
              phase_diff -= phase_diff >> PHASE_DIFF_AVG;
              phase_diff += (int32_t)(raw_phase_diff);
              ref_tracked = TRUE;
              vco_tracked = TRUE;
            }
          }
        }
        //previous_falling_ref = current_falling_ref;
      } //if(TA1CCTL1 & CCI)
      TA1IV &= ~TA1IV_TACCR1;
      break;
    case TA1IV_TACCR2: // 0x04
      if(TA1CCTL2 & CCI)
      {
        // rising edge of controlled signal from VCO
        current_rising_vco = TA1CCR2;
        //if(!(TA1IV & TA1IV_TACCR1))
        // it means vco is delayed, ref was just captured
        //if(TA1CCTL1 & CCI)
        //if(!(TA1IV & TA1IV_TACCR1))
        if((TA1CCTL1 & CCI) && !(TA1IV & TA1IV_TACCR1))
        {
          // phase_diff is POSITIVE
          //raw_phase_diff = current_rising_vco - (TA1IV & TA1IV_TACCR1)?TA1CCR1:current_rising_ref; // - target_phase_diff;
          raw_phase_diff = current_rising_vco - current_rising_ref; // - target_phase_diff;

          //if(abs(raw_phase_diff) < (1000 << PHASE_DIFF_AVG))
          {
            if(phase_diff == INT32_MAX) {
              phase_diff = (int32_t)(raw_phase_diff) << PHASE_DIFF_AVG;
            } else {
              phase_diff -= phase_diff >> PHASE_DIFF_AVG;
              phase_diff += (int32_t)(raw_phase_diff);
              vco_tracked = TRUE;
              ref_tracked = TRUE;
            }
          }
        }
        //previous_rising_vco = current_rising_vco;
      } else { //if(TA1CCTL2 & CCI)
        // falling edge of controlled signal from VCO
        current_falling_vco = TA1CCR2;
        //if(!(TA1IV & TA1IV_TACCR1))
        // it means vco is delayed, ref was just captured
        //if(TA1CCTL1 & CCI)
        //if(!(TA1IV & TA1IV_TACCR1))
        if((TA1CCTL1 & CCI) && !(TA1IV & TA1IV_TACCR1))
        {
          // phase_diff is POSITIVE
          //raw_phase_diff = current_rising_vco - (TA1IV & TA1IV_TACCR1)?TA1CCR1:current_rising_ref; // - target_phase_diff;
          raw_phase_diff = current_falling_vco - current_falling_ref; // - target_phase_diff;

          //if(abs(raw_phase_diff) < (1000 << PHASE_DIFF_AVG))
          {
            if(phase_diff == INT32_MAX) {
              phase_diff = (int32_t)(raw_phase_diff) << PHASE_DIFF_AVG;
            } else {
              phase_diff -= phase_diff >> PHASE_DIFF_AVG;
              phase_diff += (int32_t)(raw_phase_diff);
              vco_tracked = TRUE;
              ref_tracked = TRUE;
            }
          }
        }
        //previous_falling_vco = current_falling_vco;
      } //if(TA1CCTL2 & CCI)
      TA1IV &= ~TA1IV_TACCR2;
      break;
    case TA1IV_6: // reserved 0x06
      TA1IV &= ~TA1IV_6;
      break;
    case TA1IV_8: // reserved 0x08
      TA1IV &= ~TA1IV_8;
      break;
    case TA1IV_TAIFG: // 0x0A
      TA1IV &= ~TA1IV_TAIFG;
      break;
    default:
      break;
  }
  //LPM0_EXIT;
}

volatile uint8_t display_update = 2;
volatile uint8_t adc_update = 128;
volatile uint8_t log_update = 128;

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) watchdog_timer_ISR (void)
#else
#error Compiler not supported!
#endif
{
  __enable_interrupt();
  // with every fire of WDTimer it is :
  // 10kHz/64
  millis += 64;
  //millis += 512;
  if(millis > 10000) {
    millis -= 10000;
    ++seconds;
    trigger_flags |= 1 << TRIGGER_SEC;
    if(!(seconds%5))
      trigger_flags |= 1 << TRIGGER_SLW;
    LPM0_EXIT;
  }

  --display_update;
  if(!display_update) {
    display_update = 2;
    trigger_flags |= 1 << TRIGGER_LCD;
    LPM0_EXIT;
  }

  --adc_update;
  if(!adc_update) {
    adc_update = 128;
    trigger_flags |= 1 << TRIGGER_ADC;
    LPM0_EXIT;
  }

  if(rx_busy) {
    log_update = 8;
  }
  if(!log_update) {
    trigger_flags |= 1 << TRIGGER_LOG;
    log_update = 128;
    LPM0_EXIT;
  } else
    --log_update;

  if(!(ADC10CTL1 & ADC10BUSY)) {
      ADC10CTL0 |= ENC | ADC10ON | ADC10SC;             // Sampling and conversion start
  }

  trigger_flags |= 1 << TRIGGER_PID;
  LPM0_EXIT;
}
