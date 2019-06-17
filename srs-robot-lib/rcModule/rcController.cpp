#include "rcController.hpp"
#include <Arduino.h>

void rcController::init(int pin1, int pin2) {
    pinMode(pin1, INPUT);
    pinMode(pin2, INPUT);
    this->ch1 = 0;
    this->ch2 = 0;
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->isEnabled = 0;
    if (pulseIn(this->pin1, HIGH, 25000) > 100) {
        this->isEnabled = 1;
    } else {
        this->isEnabled = 0;
    }
}

int rcController::getCh1() {
    return this->ch1;
}


int rcController::getCh2() {
    return this->ch2;
}

int rcController::isRcEnabled() {
    return this->isEnabled;
}

void rcController::scan() {
    if (this->isEnabled == 1) {
        this->ch1 = pulseIn(this->pin1, HIGH, 25000);
        this->ch2 = pulseIn(this->pin2, HIGH, 25000);
    }
}