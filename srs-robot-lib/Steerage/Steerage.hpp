#ifndef Steerage_hpp
#define Steerage_hpp


#include <inttypes.h>
#include <PWMServo.h>


class Steerage {
    public:
        Steerage();
        void init(uint8_t pin, int16_t offset, int16_t maxAngle, uint16_t minPulse, uint16_t maxPulse);
        void set(int16_t value);
        int16_t get();
        static void test(uint8_t pin, int16_t offset, int16_t max, uint16_t minPulse, uint16_t maxPulse);
        void start();
        void stop();

    private:
        uint8_t pin;
        int16_t angle;
        int16_t offset;
        int16_t maxAngle;
        uint16_t minPulse;
        uint16_t maxPulse;
        PWMServo ps;
};


#endif