#include "adc.h"

// Q8.8
volatile uint16_t adc_value = 0;
volatile uint16_t adc_values[16];
volatile uint16_t pd_value = 0;
volatile uint16_t pd_values[16];
volatile uint16_t int_temp_sensor_value = 0;
volatile uint16_t int_temp_sensor_values[16];

uint16_t getOCXOTemperature() {
  return ((uint32_t)adc_value * 0x0180 * 100) >> 14;
};

uint16_t getIntTemperature() {
  //uint16_t temp_rec = ((uint32_t)n_scaled - 252) * 282;
  //uint16_t temp_rec = (((uint32_t)n_scaled - 252) * 9014) >> 5;
  //                          Q4.12            252.416 << 4    281.69 << 4
  uint16_t t_scaled = ((uint32_t)int_temp_sensor_value * 0x0018) >> 6;
  return (((uint32_t)t_scaled  -    4039)      *    4507)     >> 8;
};

uint16_t getPhaseDet() { // 3.5V in Q8.8
  return ((uint32_t)pd_value * 0x0397) >> 10;
};

void initADC() {
  // ADC10
  P1SEL |= BIT3 | BIT6;

  ADC10CTL1 &= ~ENC;
  ADC10CTL1 |= ADC10DIV_3 | INCH_3;

// VR+ = VREF+ and VR- = AVSS
// 64 x ADC10CLKs
// Multiple SampleConversion
// ADC10 Reference on 1.5V (~REF2_2V)
// ADC10 On/Enable
// ADC interrupt enable
  ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;

  ADC10AE0 |= BIT3 | BIT6;           // ADC input enable P1.3 and P1.6

  ADC10DTC1 = 16;             // 16 conversions
  ADC10SA = (uint16_t)adc_values;
};

// ADC10 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(ADC10CTL1 & INCH_15) {
    case INCH_3:
      ADC10CTL0 &= ~ENC;
      adc_value = 0;
      for(int i = 0; i < 16; ++i)
        adc_value += adc_values[i];
      //__delay_cycles(128);
      ADC10CTL0 = SREF_0 | ADC10SHT_3 | MSC | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_6;
      ADC10SA = (uint16_t)pd_values;
    break;
    case INCH_6:
      ADC10CTL0 &= ~ENC;
      pd_value = 0;
      for(int i = 0; i < 16; ++i)
        pd_value += pd_values[i];
      // output PD input on PWM output P2.5
      TA1CCR2 = pd_value << 2;
      //__delay_cycles(128);
      ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_10;
      ADC10SA = (uint16_t)int_temp_sensor_values;
    break;
    case INCH_10:
      int_temp_sensor_value = 0;
      for(int i = 0; i < 16; ++i)
        int_temp_sensor_value += int_temp_sensor_values[i];
      ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_3;
      ADC10SA = (uint16_t)adc_values;
    break;
  }

  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
