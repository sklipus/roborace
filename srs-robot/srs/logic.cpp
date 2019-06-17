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
#include "PID_v1.h"

bool enabledLogic = false;

int goBackTimeCounter = 0;


int8_t speed;
int16_t angle;
uint16_t breakCounter = 0;


#define MAX_SPEED 0


void logicInit(bool enable) {
    enabledLogic = enable;
    if (enabledLogic) {
    }
}

void logic() {
    if (!enabledLogic) {
        return;
    }
    int r90 =   Sensors::data[0];
    int r0 =    Sensors::data[1];
    int l45  =  Sensors::data[2];

    int r45  =  Sensors::data[3];
    int l0 =    Sensors::data[4];
    int l90 =   Sensors::data[5];

    if ((l0 > 90) && (r0 > 90)) {
        angle = map(r45+r90-l45-l90, -300, 300, -100, 100);
        speed = 3;
    } else if ((l0 > 65) && (r0 > 65))  {
        // middle speed mode
        angle = map(r45+r90-l45-l90, -300, 300, -300, 300);
        speed = 1;
    } 
    else {
        //slow speed
        angle = ((r90) > (l90)) ? 90 : -90;
        speed = 1;
    }

    ///////////////////////// works fine
    // if ((l0 > 50) && (r0 > 50))  {
    //     // middle speed mode
    //     angle = map(r45+r90-l45-l90, -300, 300, -100, 100);
    //     speed = 2;
    // } 
    // else {
    //     //slow speed
    //     angle = ((r90) > (l90)) ? 90 : -90;
    //     speed = 1;
    // }

    static int goBackTimeCounter = 0;
    if ((l0 < 10 || r0 < 10 || r45 < 10 || l45 < 10) && hal->mode == FORWARD) {
        goBackTimeCounter++;
            if (goBackTimeCounter > 10) {
            goBackTimeCounter = 0;
            hal->mode = GO_BACK;
        }
    } else {
        goBackTimeCounter = 0;
    }

    if (hal->mode == FORWARD) {
        hal->s1.set(angle);
        hal->m1.setSpeed(speed);
    }
}








// int getNormalFull(int k) {
//   int r0  =  Sensors::data[SEN_R0];
//   int r45 =  Sensors::data[SEN_R45];
//   int r90 =  Sensors::data[SEN_R90];
//   int l0  =  Sensors::data[SEN_L0];
//   int l45 =  Sensors::data[SEN_L45];
//   int l90 =  Sensors::data[SEN_L90];;
//   long a = ( r90 + r0 + r45 - l0 - l45 - l90) ;
//   long b = ( r90 + r0 + r45 + l0 + l45 - l90);
//   long c = 0;
//   if (b != 0) {
//     c = (a*k/b);
//   }
//   return c;
// }

// int getNormal(int k) {
//     int r0  =  Sensors::data[SEN_R0];
//     int r45 =  Sensors::data[SEN_R45];
//     int r90 =  Sensors::data[SEN_R90];
//     int l0  =   Sensors::data[SEN_L0];
//     int l45 =  Sensors::data[SEN_L45];
//     int l90 =  Sensors::data[SEN_L90];;
//     long a = ( r90 + r0 + r45 - l0 - l45 - l90) ;
//     long b = ( r90 + r0 + r45 + l0 + l45 - l90);
//     long c = 0;
//     if (b != 0) {
//       c = (a*k/b);
//     }
//     return c;
// }

// long mapVal(long x, long in_min, long in_max, long out_min, long out_max) {
//   if ((in_max - in_min) != 0) {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//   } else return 0;
// }

// long mapAngle100(long vec, long maxAngle) {
//     return constrain(mapVal(vec,-100,100,-maxAngle,maxAngle),-maxAngle,maxAngle);
// }
