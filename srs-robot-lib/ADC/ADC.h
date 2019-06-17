//ADC
#define ADC_VALUE 0
#define IS_ADC_VALUE_READY 1

#define FIRST_ADC_INPUT 0
#define LAST_ADC_INPUT 7

#define NUMBER_OF_READING 2

// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

void ADC_setup();
extern volatile uint16_t adc_data[LAST_ADC_INPUT-FIRST_ADC_INPUT+1][2];

extern uint8_t getGP2Y0A60SZLF(uint8_t value);