#include <Arduino.h>
#include "config.h"
#include "HAL.hpp"

void driveByRadioCommands() {
    hal->rc.scan();
    


    int ch2 = hal->rc.getCh2();
    hal->m1.setSpeed(map(ch2,700,2300,-9,9));

    int ch1 = hal->rc.getCh1();
    hal->s1.set(map(ch1,1000,2000,-50,50));
    //hal->m1.setSpeed(3);
}
