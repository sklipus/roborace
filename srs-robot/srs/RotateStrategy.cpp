#include <Arduino.h>
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include <Motor.hpp>
#include "sensors.h"
#include "Messaging.hpp"
#include "logic.hpp"
#include "HAL.hpp"
#include "RotateStrategy.hpp"

int rotateMotorCounter = 0;

static uint32_t rotateStartTime1 = 0;

void rotateRobot() {
    if (hal->mode == ROTATE) {
        uint32_t diff, currentTime;
        currentTime = millis();
        rotateStartTime1 = rotateStartTime1 != 0 ? rotateStartTime1 : currentTime;
        diff = currentTime - rotateStartTime1;

        if (diff < 1000) {

            hal->s1.set(0);
            hal->m1.setSpeed(0);

        } else if (diff < 2000) {

            hal->s1.set(-50);
            hal->m1.setSpeed(-3);

        } else if (diff < 3000) {

            hal->s1.set(50);
            hal->m1.setSpeed(1);

        } else {

            rotateStartTime1 = 0;
            diff = 0;
            hal->mode = FORWARD;

        }
    }
}