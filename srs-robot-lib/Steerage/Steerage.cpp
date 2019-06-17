#include <avr/interrupt.h>
#include <Arduino.h> 
#include <PWMServo.h>
#include "Steerage.hpp"


Steerage::Steerage(){

}


void Steerage::init(uint8_t pin, int16_t offset, int16_t max, uint16_t minPulse, uint16_t maxPulse) {
    this->pin = pin;
    this->minPulse = minPulse;
    this->maxPulse = maxPulse;
    this->offset = offset;
    this->maxAngle = max;
    this->angle = 0;
    this->ps = PWMServo();
    this->start();
}

void Steerage::start() {
    this->ps.attach(this->pin, this->minPulse, this->maxPulse);
    this->ps.write(this->offset + this->angle);
}


void Steerage::stop() {
    this->ps.detach();
}


void Steerage::set(int16_t value) {
    this->angle = constrain(value, - this->maxAngle, this->maxAngle);
    this->ps.write((int16_t)this->offset + this->angle);
}


int16_t Steerage::get() {
    return this->angle;
}

void Steerage::test(uint8_t pin, int16_t offset, int16_t max, uint16_t minPulse, uint16_t maxPulse) {
    Steerage s1;
    s1.init(pin, offset, max, minPulse, maxPulse);
    while(1) {
        s1.set(50);
        delay(1000);
        s1.set(-50);
        delay(1000);
    }
}

