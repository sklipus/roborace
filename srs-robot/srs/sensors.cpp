#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include "config.h"
#include "sensors.h"
#include "HAL.hpp"


uint8_t Sensors::data[NUMBER_OF_SENSORS];
uint8_t Sensors::sensorsHistory[NUMBER_OF_SENSORS][MEDIAN_SIZE];
int16_t Sensors::deltaOfData[NUMBER_OF_SENSORS][2];
uint8_t Sensors::maxDistance = 0;
uint8_t Sensors::minDistance = 0;

uint8_t Sensors::get_voltage(uint8_t index) {
    // ADC VALUE 10 BIT
    return adc_data[index][ADC_VALUE] >> (ADC_BIT_SIZE - sizeof(uint8_t));
}


uint8_t Sensors::calcMedian(uint8_t index, uint8_t newValue) {
    static uint8_t tempArray[MEDIAN_SIZE];
    uint8_t *ar = sensorsHistory[index];
    for (int i = 0; i < MEDIAN_SIZE-1; i++) {
        ar[i] = ar[i+1];
    }
    ar[(MEDIAN_SIZE-1)] = newValue;

    for(int i = 0; i < MEDIAN_SIZE; i++ )
        tempArray[i] = ar[i];

    for( int i = 0; i < (MEDIAN_SIZE-1); i++ ) {
        uint8_t swapped = 0;
        for( int j=0; j < (MEDIAN_SIZE-1); j++ ) {
            if (tempArray[j] > tempArray[j+1]) {
                int b = tempArray[j];
                tempArray[j] = tempArray[j+1];
                tempArray[j+1] = b;
                swapped = 1;
            }
        }
        if(!swapped)
            break;
    }
    return tempArray[(MEDIAN_SIZE-1)/2];
}


void Sensors::updateSensors() {
    uint8_t j = 0;
    for(uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {

        j = i == 4 ? 6 : i;
        j = i == 5 ? 7 : j;

        if (adc_data[j][IS_ADC_VALUE_READY]) {
            Sensors::data[i] = calcMedian(i, getGP2Y0A60SZLF(adc_data[j][ADC_VALUE] / 10));
            adc_data[j][IS_ADC_VALUE_READY] = 0;
        }
    }

    // find max and min
    Sensors::maxDistance = Sensors::minDistance = Sensors::data[0];
    for(uint8_t i=1; i < NUMBER_OF_SENSORS; i++) {
        Sensors::maxDistance = Sensors::data[i] > Sensors::maxDistance ? Sensors::data[i] : Sensors::maxDistance;
        Sensors::minDistance = Sensors::data[i] < Sensors::minDistance ? Sensors::data[i] : Sensors::minDistance;
    }
}


void Sensors::updateDeltaOfSensors() {
    for(uint8_t i=0; i < NUMBER_OF_SENSORS; i++) {
        Sensors::deltaOfData[i][DELTA_OF_SENSORS] = ((int8_t)Sensors::data[i] - (int8_t)Sensors::deltaOfData[i][PREV_VALUE_OF_SENSORS]);
        Sensors::deltaOfData[i][PREV_VALUE_OF_SENSORS] = Sensors::data[i];
    }
}


void Sensors::checkIfRobotIsBlocked() {
    static int8_t counter = 0;
    uint8_t curSpeed = hal->m1.getRealSpeed();
    int8_t setSpeed = hal->m1.getSpeed();
    if (hal->mode == FORWARD) {
        switch (setSpeed)
        {
            case 1:
                counter = curSpeed < 8 ? counter+1 : 0;
                break;
            case 2:
                counter = curSpeed < 13 ? counter+1 : 0;
                break;
            case 3:
                counter = curSpeed < 16 ? counter+1 : 0;
                break;
            case 4:
                counter = curSpeed < 16 ? counter+1 : 0;
                break;
            case 5:
                counter = curSpeed < 20 ? counter+1 : 0;
                break;
            case 6:
                counter = curSpeed < 50 ? counter+1 : 0;
                break;
            case 7:
                counter = curSpeed < 50 ? counter+1 : 0;
                break;
            case 8:
                counter = curSpeed < 50 ? counter+1 : 0;
                break;
            case 9:
                counter = curSpeed < 50 ? counter+1 : 0;
                break;
            default:
                counter = 0;
                break;
        }

        if (counter > 20) {
            counter = 0;
            hal->mode = GO_BACK_SAFETY;
        }
    }

}
