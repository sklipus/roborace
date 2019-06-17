#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include <Motor.hpp>
#include "config.h"
#include "sensors.h"
#include "Messaging.hpp"
#include "logic.hpp"
#include "HAL.hpp"
#include "GoBackStrategy.hpp"

int goBackMotorCounter = 0;

static unsigned long startTime = 0;

void moveRobotBack() {
    if (hal->mode == GO_BACK) {
        uint32_t currentTime = millis();
        startTime = startTime != 0 ? startTime : currentTime;
        uint32_t diff = currentTime - startTime;
        if (diff < 350) {
            hal->s1.set(0);
            hal->m1.setSpeed(0);
        } else if (diff < 1000) {
            hal->s1.set(0);
            hal->m1.setSpeed(-3);
        } else if (diff < 1100) {
            hal->s1.set(0);
            hal->m1.setSpeed(0);
        } else {
            startTime = 0;
            hal->mode = FORWARD;
        }
    }
}


void moveRobotBackSafety() {
    if (hal->mode == GO_BACK_SAFETY) {
        uint32_t currentTime = millis();
        startTime = startTime != 0 ? startTime : currentTime;
        uint32_t diff = currentTime - startTime;
        if (diff < 1000) {
            hal->s1.set(0);
            hal->m1.setSpeed(0);
        } else if (diff < 1500) {
            hal->s1.set(0);
            hal->m1.setSpeed(-3);
        } else if (diff < 2000) {
            hal->s1.set(0);
            hal->m1.setSpeed(0);
        } else {
            startTime = 0;
            hal->mode = FORWARD;
        }
    }
}