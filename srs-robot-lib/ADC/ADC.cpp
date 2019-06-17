#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ADC.h"
#include "GP2Y0A60SZLF.h"


volatile uint16_t adc_data[LAST_ADC_INPUT-FIRST_ADC_INPUT+1][2];


uint8_t getGP2Y0A60SZLF(uint8_t value) {
  return GP2Y0A60SZLF[value];
}


void ADC_setup() {
  cli();
  // ADC initialization
  // ADC Clock frequency: 125.000 kHz
  // ADC Voltage Reference: AVCC pin
  // ADC Auto Trigger Source: Free Running
  // Digital input buffers on ADC0: Off, ADC1: Off, ADC2: Off, ADC3: Off
  // ADC4: Off, ADC5: Off
  DIDR0=(1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
  ADMUX=FIRST_ADC_INPUT | ADC_VREF_TYPE;
  ADCSRA=(1<<ADEN) | (1<<ADSC) | (1<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
  sei();
}

// ADC interrupt service routine
// with auto input scanning

ISR(ADC_vect, ISR_BLOCK) {
  static uint8_t read_counter = 0;
  if (++read_counter >= NUMBER_OF_READING) {
    read_counter = 0;
    static uint8_t uc_input_index = 0;
    static uint8_t uc_input_value = 0;
    // Read the AD conversion result
    adc_data[uc_input_index][ADC_VALUE] = ADCW;
    adc_data[uc_input_index][IS_ADC_VALUE_READY] = 1;
    // Select next ADC input
    if (++uc_input_index > (LAST_ADC_INPUT-FIRST_ADC_INPUT))
      uc_input_index=0;
    #ifdef ENABLE_WIRE
      uc_input_value = uc_input_index == 4 ? 6 : uc_input_index;
      uc_input_value = uc_input_index == 5 ? 7 : uc_input_value;
      ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE) + uc_input_value;
    #else
      uc_input_value = uc_input_index;
      ADMUX=(FIRST_ADC_INPUT | ADC_VREF_TYPE) + uc_input_value;
    #endif
  }
  // Start the AD conversion
  ADCSRA|=(1<<ADSC);
}
