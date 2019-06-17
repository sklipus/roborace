#ifndef HAL_hpp
#define HAL_hpp

#include <Arduino.h>
#include <Steerage.hpp>
#include <Motor.hpp>
#include <rcController.hpp>

enum {FORWARD , GO_BACK, ROTATE, GO_BACK_SAFETY};

class HighAlgoritmicLayer {
    public:
        Steerage s1;
        Motor m1;
        rcController rc;
        int mode;
        HighAlgoritmicLayer () {
            Motor m1();
            Steerage s1();
            rcController rc();
            mode = FORWARD;
        }
};

extern HighAlgoritmicLayer *hal;

#endif