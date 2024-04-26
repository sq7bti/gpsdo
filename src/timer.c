#include "timer.h"

static volatile uint8_t ticks = 0, clicks = 0;
static volatile uint32_t millis = 0, seconds = 0;
static volatile uint16_t ocxo_count = 0;
static volatile uint16_t capture_count = 0;
static volatile uint16_t capture_overflow = 0;

volatile uint16_t previous_rising_ref = 0;
volatile uint16_t current_rising_ref = 0;
volatile uint16_t period_ref = 0;
volatile uint16_t previous_rising_vco = 0;
volatile uint16_t current_rising_vco = 0;
volatile uint16_t period_vco = 0;

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
  // 10kHz comparator P2.1 and P2.4
  P2SEL |= BIT1 | BIT4;                       // Use P2.1 and P2.1 as Timer1_A0 inputs
	P2SEL2 &= ~(BIT1 | BIT4);                   //
	P2DIR &= ~(BIT1 | BIT4);                    // inputs
	P2OUT &= ~(BIT1 | BIT4);                    //
	P2REN |= BIT1 | BIT4;                       // Enable pull down resistor to reduce stray counts

  // pwm ouput on P1.6
  P1SEL |= BIT6;                              // Use P1.6 as TimerA1 output
	P1SEL2 &= ~BIT6;                            // Timer1_A3.TAO
	P1DIR |= BIT6;                              // output

  // capture mode; rising edge; input select CCIxA; enable interrupts
  //TA0CCTL0 = CAP | CM_1 | CCIS_0 | CCIE;

  // CCR1 - PWM output on P1.6
  TA0CCTL1 = OUTMOD_7; // reset/set output
  TA0CCR1 = 0x8000;   // 50% PWM

  // TACLK - connected 10MHz from OCXO; continuous up until 0xFFFF; clear
  //TA0CTL = TASSEL_0 | MC_2 | TACLR;
  // SMCLK - 16MHz; continuous up until 0xFFFF; clear
  TA0CTL = TASSEL_2 | MC_2 | TACLR;


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
__interrupt void Timer0_A1_iSR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A1_iSR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA0IV, 0x0A)) {
    case TA0IV_NONE: // 0x00
      break;
    case TA0IV_TACCR1: // TACCR1 CCIFG
      break;
    case TA0IV_TACCR2: // TACCR2 CCIFG
      break;
    case TA0IV_6: // reserved
      break;
    case TA0IV_8: // reserved
      break;
    case TA0IV_TAIFG: // TAIFG
      break;
    default:
      break;
  }
  __bic_SR_register_on_exit(LPM1_bits + GIE); // Enter LPM3
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
      // rising edge of reference signal from GPGSA
      current_rising_ref = TA1CCR1;
      period_ref = current_rising_ref - previous_rising_ref;
      previous_rising_ref = current_rising_ref;
      break;
    case TA1IV_TACCR2: // 0x04
      // rising edge of reference signal from GPGSA
      current_rising_vco = TA1CCR2;
      period_vco = current_rising_vco - previous_rising_vco;
      previous_rising_vco = current_rising_vco;
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
  __bic_SR_register_on_exit(LPM1_bits + GIE); // Enter LPM3
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

  if(!(ADC10CTL1 & ADC10BUSY)) {
      ADC10CTL0 |= ENC | ADC10ON | ADC10SC;             // Sampling and conversion start
  }
  __bic_SR_register_on_exit(LPM1_bits + GIE); // Enter LPM3
}
