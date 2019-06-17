#include <Arduino.h>
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include <Observer.h>
#include <Steerage.hpp>
#include <Motor.hpp>
#include "sensors.h"
#include "Messaging.hpp"
#include "rcController.hpp"
#include "logic.hpp"
#include "RadioController.hpp"
#include "HAL.hpp"
#include "GoBackStrategy.hpp"
#include "PID_v1.h"
#include "pidBasedStrategy.hpp"
#include "RotateStrategy.hpp"
#include <ArduinoJson.h>
#include "CheckRobotDirection.hpp"


void drive() {
    hal->m1.drive();
}

void calcSpeed() {
    hal->m1.calcRealSpeed();
}


void setup() {
    ADC_setup();

    hal->s1.init(SERVO_PIN, 90, 50, 544, 2400);
    //Steerage::test(SERVO_PIN, 90, 50, 544, 2400);

    uint16_t arrF[10] = FORWARD_SPEED;
    uint16_t arrB[10] = BACKWARD_SPEED;
    hal->m1.init(MOTOR_PIN, false, arrB, arrF);

    Messaging::init();

    hal->rc.init(4,5);
    hal->m1.setSpeed(2);
    hal->m1.drive();

    #ifndef PID_MODE
        logicInit(true);
    #else
        initPidStrategy(true);
    #endif
}


Observer updateSensor(Sensors::updateSensors);
Observer logicObs(logic);
Observer motorObs(drive);
Observer calcSpeedObs(calcSpeed);
Observer updateDeltaOfSensors(Sensors::updateDeltaOfSensors);
Observer rcObs(driveByRadioCommands);
Observer goBackObs(moveRobotBack);
Observer goBackSafeObs(moveRobotBackSafety);
Observer checkRobotBlockObs(Sensors::checkIfRobotIsBlocked);
Observer rotateRobotObs(rotateRobot);
Observer pidStrategyObs(pidStrategy);
Observer checkCorrectTurnObs(checkCorrectTurn);


volatile uint32_t timer_current_millis = 0;


void loop() {
    updateSensor.doCode(                           0, 5);
    #ifndef PID_MODE
        calcSpeedObs.doCode(         WEEL_COUNTER_PERIOD, 5);
    #endif
    if (hal->rc.isRcEnabled() == 1) {
        rcObs.doCode(                             50, 5);
    } else {

        goBackObs.doCode(                         10, 5);
        goBackSafeObs.doCode(                     10, 5);
        rotateRobotObs.doCode(                    10, 5);

        #ifndef PID_MODE

            logicObs.doCode(                       0, 5);
            checkCorrectTurnObs.doCode(           50, 5);

        #else
            pidStrategyObs.doCode(                 0, 5);
        #endif
    }

    motorObs.doCode(                              0, 5);
    updateDeltaOfSensors.doCode( WEEL_COUNTER_PERIOD, 5);

    #ifndef PID_MODE
        checkRobotBlockObs.doCode(   WEEL_COUNTER_PERIOD, 5);
    #endif
}












/*
#include <Arduino.h>
#include <Motor.hpp>


Motor m1;
uint16_t forwardSpeeds[10]  = {1480, 1520, 1570, 1600, 1630, 1730, 2000, 2000, 2000, 2500};
uint16_t backwardSpeeds[10] = {1420, 1380, 1300, 1250, 1200, 1150, 1100, 1000, 800, 500};

uint16_t period = 20;
uint16_t lastTime = 0;


void setup() {
    Serial.begin(9600);
    m1.init(10, false, backwardSpeeds, forwardSpeeds);
    delay(4000);
    m1.setSpeed(2);
    Serial.println("12121212");
}


void loop() {
    while(1) {
        m1.drive();
        if (lastTime + period < millis()) {
            lastTime = millis();
            m1.calcRealSpeed();
            Serial.println(m1.getRealSpeed());
        }
    }
    // put your main code here, to run repeatedly:
}
*/