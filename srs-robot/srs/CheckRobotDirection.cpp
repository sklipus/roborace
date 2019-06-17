#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include <Motor.hpp>
#include "config.h"
#include "HAL.hpp"
#include "CheckRobotDirection.hpp"

long rotetedAngle = 0;
#define MAXCOUNTANGLE 1500
void checkCorrectTurn() {
    int stAngle = hal->s1.get();
    if (hal->mode == FORWARD) {
        if (rotetedAngle + stAngle >= MAXCOUNTANGLE) {
            rotetedAngle = MAXCOUNTANGLE;
        } else if (rotetedAngle + stAngle <= -MAXCOUNTANGLE) {
            rotetedAngle = -MAXCOUNTANGLE;
            hal->mode = ROTATE;
            rotetedAngle = MAXCOUNTANGLE;
        } else {
            rotetedAngle += stAngle;
        }
    }
}