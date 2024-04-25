#include "timer.h"

static volatile uint8_t ticks = 0, clicks = 0;
static volatile uint32_t millis = 0, seconds = 0;
static volatile uint16_t ocxo_count = 0;
static volatile uint16_t capture_count = 0;
static volatile uint16_t capture_overflow = 0;

uint8_t getticks() { return ticks; };
uint8_t getclicks() { return clicks; };
uint32_t getMillis() { return millis/10; };
uint32_t getSeconds() { return seconds; };
uint16_t getOCXO() { return ocxo_count; };
uint32_t getCAP() { return ((uint32_t)capture_overflow << 16) + capture_count; };

extern uint16_t* adc_values;
extern uint16_t* int_temp_sensor_values;

void initTIMER(void)
{
  // timer 0 used
  P1SEL |= BIT0;                              // Use P1.0 as TimerA0 input
	P1SEL2 &= ~BIT0;                            //
	P1DIR &= ~BIT0;                             // input
	P1OUT &= ~BIT0;                             //
	P1REN |= BIT0;                              // Enable pull down resistor to reduce stray counts

  // capture mode; rising edge; input select CCIxA; enable interrupts
  TA0CCTL0 = CAP | CM_1 | CCIS_0 | CCIE;
  // TACLK - connected 10MHz from OCXO; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_0 | MC_2 | TACLR;

  // timer 1
  P2SEL |= BIT0 | BIT5;                              // Use P2.0 as TimerA1 output
	P2SEL2 &= ~(BIT0 | BIT5);                            // Timer1_A3.TAO
	P2DIR |= BIT0 | BIT5;                              // output
	//P2OUT |= BIT0;                              // Enable pull down resistor to reduce stray counts

  // timer 1 used to track 10kHz from reference (GPS) and VCO (OCXO)
  TA1CTL = TASSEL_2 | MC_2; // SMCLK; continuous

  // CCR0 - PWM output on P2.0 (or P2.3)
  //TA1CCTL0 = OUTMOD_7; // set/reset output
  //TA1CCR0 = 0x0000;   // 50% PWM

  // CCR2 - PWM output on P2.5 (or P2.4)
  TA1CCTL2 = OUTMOD_3; // set/reset output
  TA1CCR2 = 0x8000;   // 50% PWM
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
  switch(__even_in_range(TA0IV, 0x0A)) {
    case 2: // TACCR1 CCIFG
      capture_count = TACCR0;
      break;
    case 10: // TAIFG
      ++capture_overflow;
      break;
  }
//  if(TACCTL0 & CCIFG) {
//    capture_count |= TA0R;
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
  switch(__even_in_range(TA1IV, 0x0A)) {
    case 2: // TACCR1 CCIFG
      capture_count = TACCR0;
      break;
    case 10: // TAIFG
      ++capture_overflow;
      break;
  }
//  if(TACCTL0 & CCIFG) {
//    capture_count |= TA0R;
//  }
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
  ocxo_count = TA0R;
  capture_overflow = 0;
  TA0CTL |= TACLR;
  // with every fire of WDTimer it is :
  // 10kHz/64
  millis += 64;
  //millis += 512;
  if(millis > 10000) {
    millis -= 10000;
    ++seconds;
  }

//  if(!(ADC10CTL1 & ADC10BUSY)) {
    /*  switch(ADC10CTL1 & INCH_15) {
      case INCH_10:
        ADC10CTL1 &= ~(INCH_15);
        ADC10SA = (uint16_t*)adc_values;
        ADC10CTL1 |= INCH_3;
        break;
        case INCH_3:
        ADC10CTL1 &= ~(INCH_15);
        ADC10SA = (uint16_t*)int_temp_sensor_values;
        ADC10CTL1 |= INCH_10;
        break;
      }*/
      //ADC10CTL1 &= ~(INCH_15);
      //ADC10SA = (uint16_t*)adc_values;
      //ADC10CTL1 |= INCH_3;
      ADC10CTL0 |= ENC | ADC10ON | ADC10SC;             // Sampling and conversion start
//    }
}
