#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <ADC.h>
#include <Motor.hpp>
#include "config.h"
#include "sensors.h"
#include "Messaging.hpp"
#include "HAL.hpp"
#include "PID_v1.h"

bool enabledPidStrategy = false;
long lastExecTime = 0;
int16_t angleRobot;
double mSetpoint, mInput, mOutput;
double mConsKp=0.1, mConsKi=0.01, mConsKd=0.1;
int speedRobot;
PID motorPID(&mInput, &mOutput, &mSetpoint, mConsKp, mConsKi, mConsKd, DIRECT);

void initPidStrategy(bool enable) {
    enabledPidStrategy = enable;
    if (enabledPidStrategy) {
        motorPID.SetOutputLimits(-1, 4);
        motorPID.SetMode(AUTOMATIC);
    }
}

void pidStrategy () {
    if (!enabledPidStrategy) {
        return;
    }

    int r90 =   Sensors::data[0];
    int r0 =    Sensors::data[1];
    int l45  =  Sensors::data[2];

    int r45  =  Sensors::data[3];
    int l0 =    Sensors::data[4];
    int l90 =   Sensors::data[5];

    if ((l0 > 90) && (r0 > 90)) {
        angleRobot = map(r45+r90-l45-l90, -300, 300, -100, 100);
        speedRobot = 50;
    } else if ((l0 > 65) && (r0 > 65))  {
        // middle speed mode
        angleRobot = map(r45+r90-l45-l90, -300, 300, -300, 300);
        speedRobot = 25;
    } 
    else {
        //slow speed
        angleRobot = ((r90) > (l90)) ? 90 : -90;
        speedRobot = 20;
    }

    mConsKp = (double)((double)10/(double)100.0);
    mConsKi = (double)((double)1/(double)1000.0);
    mConsKd = (double)((double)1/(double)1000.0);
    motorPID.SetTunings(mConsKp, mConsKi, mConsKd);

    if (hal->mode == FORWARD) {
        hal->s1.set(angleRobot);
    }
    
    static int goBackTimeCounter = 0;
    if (l0 < 10 || r0 < 10 || r45 < 10 || l45 < 10) {
        goBackTimeCounter++;
        if (goBackTimeCounter > 10) {
            goBackTimeCounter = 0;
            hal->mode = GO_BACK;
        }
    } else {
        goBackTimeCounter = 0;
    }

    long currentTime = millis();
    if (((lastExecTime + WEEL_COUNTER_PERIOD) < currentTime) && (hal->mode == FORWARD)) {
        hal->m1.calcRealSpeed();
        mInput = hal->m1.getRealSpeed();
        mSetpoint = speedRobot;
        motorPID.Compute();
        mOutput = mOutput == 0 ? 1 : mOutput;
        hal->m1.setSpeed(mOutput);
        lastExecTime = currentTime;
    }
    
}

void calculateCurrentSpeed() {

}

/*

    motorPID.SetOutputLimits(-9, 9);
    motorPID.SetMode(AUTOMATIC);

    double mSetpoint, mInput, mOutput;
double mConsKp=0.1, mConsKi=0, mConsKd=0.01;
PID motorPID(&mInput, &mOutput, &mSetpoint, mConsKp, mConsKi, mConsKd, DIRECT);
void calcSpeed() {
    hal->m1.calcRealSpeed();
    mSetpoint = speed;
    mInput = hal->m1.getRealSpeed();
    if (hal->goBack == false) {
        motorPID.Compute();
        hal->m1.setSpeed(mOutput);
    }
    hal->m1.drive();
    
}


// aggKp = Messaging::params[0]/10;
    // aggKi = Messaging::params[0]/100;
    // aggKd = Messaging::params[0]/100;

    // if ((l0 > 50) && (r0 > 50)) {
    //     //Input = r45+r90-l45-l90;
    //     anglePID.SetTunings(aggKp, aggKp, aggKp);
    // } else {
    //     //Input = r90-l90;
    //     anglePID.SetTunings(aggKp, aggKi, aggKd);
    // }
    // Input = r90-l90;
    // anglePID.Compute();
    // speed = 1;
    // angle = (int16_t) Output;
*/