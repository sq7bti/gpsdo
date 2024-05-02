#include "adc.h"

static volatile uint8_t ticks = 0, clicks = 0;

// Q8.8
volatile uint16_t ocxo_temp_raw_value = 0;
volatile uint16_t ocxo_temp_raw_values[16];
volatile uint16_t phase_comp_raw_value = 0;
volatile uint16_t phase_comp_raw_values[16];
volatile uint16_t int_temp_sensor_value = 0;
volatile uint16_t int_temp_sensor_values[16];

uint16_t getOCXOTemperature() {
  return ((uint32_t)ocxo_temp_raw_value * 0x0018 * 100) >> 10;
};

uint16_t getIntTemperature() {
  //uint16_t temp_rec = ((uint32_t)n_scaled - 252) * 282;
  //uint16_t temp_rec = (((uint32_t)n_scaled - 252) * 9014) >> 5;
  //        Q20.12            252.416 << 4    281.69 << 4
  uint32_t t_scaled = ((uint32_t)int_temp_sensor_value * 0x0018) >> 8;
  return ((t_scaled  -    4039)      *    4507)     >> 8;
};

uint16_t getPhaseDet() {
  // Vcc 3.59V in Q8.8
  //return ((uint32_t)phase_comp_raw_value * 0x0397) >> 10;
  // Vcc 5.035 in Q8.8
  return ((uint32_t)phase_comp_raw_value * 0x0509) >> 10;
};

uint8_t getticks() { return ticks; };

void initADC() {
  // ADC10
  P1SEL |= BIT3 | BIT4;

  ADC10CTL1 &= ~ENC;
  ADC10CTL1 |= ADC10DIV_3 | INCH_3;

// VR+ = VREF+ and VR- = AVSS
// 64 x ADC10CLKs
// Multiple SampleConversion
// ADC10 Reference on 1.5V (~REF2_2V)
// ADC10 On/Enable
// ADC interrupt enable
  ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;

  ADC10AE0 |= BIT3 | BIT4;           // ADC input enable P1.3 and P1.4

  ADC10DTC1 = 16;             // 16 conversions
  ADC10SA = (uint16_t)ocxo_temp_raw_values;
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
  __enable_interrupt();
  switch(ADC10CTL1 & INCH_15) {
    case INCH_3:
      ADC10CTL0 &= ~ENC;
      if(!ocxo_temp_raw_value) {
        for(int i = 0; i < 16; ++i)
          ocxo_temp_raw_value += ocxo_temp_raw_values[i];
        //ocxo_temp_raw_value <<= 4;
        } else {
          //ocxo_temp_raw_value -= ocxo_temp_raw_value >> 4;
          ocxo_temp_raw_value = 0;
          for(int i = 0; i < 16; ++i)
            ocxo_temp_raw_value += ocxo_temp_raw_values[i];
        }
      //__delay_cycles(128);
      ADC10CTL0 = SREF_0 | ADC10SHT_3 | MSC | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_4;
      ADC10SA = (uint16_t)phase_comp_raw_values;
    break;
    case INCH_4:
      ADC10CTL0 &= ~ENC;
      phase_comp_raw_value = 0;
      for(int i = 0; i < 16; ++i)
        phase_comp_raw_value += phase_comp_raw_values[i];
      //__delay_cycles(128);
      ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_10;
      ADC10SA = (uint16_t)int_temp_sensor_values;
    break;
    case INCH_10:
      if(!int_temp_sensor_value) {
        for(int i = 0; i < 16; ++i)
          int_temp_sensor_value += int_temp_sensor_values[i];
        int_temp_sensor_value <<= 2;
      } else {
        int_temp_sensor_value -= int_temp_sensor_value >> 2;
        //int_temp_sensor_value = 0;
        for(int i = 0; i < 16; ++i)
          int_temp_sensor_value += int_temp_sensor_values[i];
      }
      ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10IE;
      ADC10CTL1 = ADC10DIV_3 | INCH_3;
      ADC10SA = (uint16_t)ocxo_temp_raw_values;
    break;
  }

//  if((ADC10CTL1 & INCH_15) == INCH_3) {
    if(!ticks) {
      ticks = 1;
      LPM0_EXIT;
    }
    --ticks;
//  }
}
