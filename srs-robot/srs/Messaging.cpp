#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Motor.hpp>
#include "config.h"
#include "Messaging.hpp"
#include "sensors.h"
#include "HAL.hpp"




// doc["RealSpeed"] = hal->m1.getRealSpeed();
    // doc["Speed"] = hal->m1.getSpeed();
    // doc["Angle"] = hal->s1.get();
    //doc["Sensor-r90-0"] = 1;//
    // doc["Sensor-r0-1"] = Sensors::data[1];
    // doc["Sensor-l45-2"] = Sensors::data[2];
    // doc["Sensor-r45-3"] = Sensors::data[3];
    // doc["Sensor-l0-4"] = Sensors::data[4];
    // doc["Sensor-l90-5"] = Sensors::data[5];
    // doc["Voltage"] =    Sensors::get_voltage(6);
    // doc["dtSensor-0"] = Sensors::deltaOfData[0][DELTA_OF_SENSORS];
    // doc["dtSensor-1"] = Sensors::deltaOfData[1][DELTA_OF_SENSORS];
    // doc["dtSensor-2"] = Sensors::deltaOfData[2][DELTA_OF_SENSORS];
    // doc["dtSensor-3"] = Sensors::deltaOfData[3][DELTA_OF_SENSORS];
    // doc["dtSensor-4"] = Sensors::deltaOfData[4][DELTA_OF_SENSORS];
    // doc["dtSensor-5"] = Sensors::deltaOfData[5][DELTA_OF_SENSORS];

char command[10];

void onReceive(int howMany) {
    int i;
    i = 0;
    while (Wire.available()) {
        command[i++] = Wire.read();
        if (i >= sizeof(command)) {
            break;
        }
    }
}

char * itoaCustom (int value, char *result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

char responce[32];
void onRequest() {
    for (int i = 0; i < 32; i++) {
      responce[i] = 0;
    }

    if (strcmp(command,"speed-----") == 0) {
        itoaCustom(hal->m1.getSpeed(), responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"realSpeed-") == 0) {
        itoaCustom(hal->m1.getRealSpeed(), responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"angle-----") == 0) {
        itoaCustom(hal->s1.get(), responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--0") == 0) {
        itoaCustom(Sensors::data[0], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--1") == 0) {
        itoaCustom(Sensors::data[1], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--2") == 0) {
        itoaCustom(Sensors::data[2], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--3") == 0) {
        itoaCustom(Sensors::data[3], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--4") == 0) {
        itoaCustom(Sensors::data[4], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"iSensor--5") == 0) {
        itoaCustom(Sensors::data[5], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-0") == 0) {
        itoaCustom(Sensors::deltaOfData[0][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-1") == 0) {
        itoaCustom(Sensors::deltaOfData[1][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-2") == 0) {
        itoaCustom(Sensors::deltaOfData[2][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-3") == 0) {
        itoaCustom(Sensors::deltaOfData[3][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-4") == 0) {
        itoaCustom(Sensors::deltaOfData[4][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    } else if (strcmp(command,"idSensor-5") == 0) {
        itoaCustom(Sensors::deltaOfData[5][DELTA_OF_SENSORS], responce, 10);
        Wire.write(responce);
    }
}


void Messaging::init() {
    Wire.begin(8);
    Wire.onRequest(onRequest);
    Wire.onReceive(onReceive);
}

    // message[(uint8_t)doc["RealSpeed"]] = hal->m1.getRealSpeed();
    // message[(uint8_t)doc["Speed"]] = hal->m1.getSpeed();
    // message[(uint8_t)doc["Angle"]] = (int8_t) hal->s1.get();
    // message[(uint8_t)doc["Sensor-r90-0"]] = (uint8_t) Sensors::data[0];
    // message[(uint8_t)doc["Sensor-r0-1"]] = (uint8_t) Sensors::data[1];
    // message[(uint8_t)doc["Sensor-l45-2"]] = (uint8_t) Sensors::data[2];
    // message[(uint8_t)doc["Sensor-r45-3"]] = (uint8_t) Sensors::data[3];
    // message[(uint8_t)doc["Sensor-l0-4"]] = (uint8_t) Sensors::data[4];
    // message[(uint8_t)doc["Sensor-l90-5"]] = (uint8_t) Sensors::data[5];
    // message[(uint8_t)doc["Voltage"]] =  (uint8_t) Sensors::get_voltage(6);
    // message[(uint8_t)doc["dtSensor-0"]] = (uint8_t) Sensors::deltaOfData[0][DELTA_OF_SENSORS];
    // message[(uint8_t)doc["dtSensor-1"]] = (uint8_t) Sensors::deltaOfData[1][DELTA_OF_SENSORS];
    // message[(uint8_t)doc["dtSensor-2"]] = (uint8_t) Sensors::deltaOfData[2][DELTA_OF_SENSORS];
    // message[(uint8_t)doc["dtSensor-3"]] = (uint8_t) Sensors::deltaOfData[3][DELTA_OF_SENSORS];
    // message[(uint8_t)doc["dtSensor-4"]] = (uint8_t) Sensors::deltaOfData[4][DELTA_OF_SENSORS];
    // message[(uint8_t)doc["dtSensor-5"]] = (uint8_t) Sensors::deltaOfData[5][DELTA_OF_SENSORS];
    // message[doc.size()+1] = '}';
    // message[0] = '{';
