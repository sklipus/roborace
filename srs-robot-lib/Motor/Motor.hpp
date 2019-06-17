#ifndef Motor_hpp
#define Motor_hpp
#include <PWMServo.h>

// motor (back wheels are powered)
#define WEEL_COUNTER_PERIOD 20

class Motor {
  public:
    Motor();
    void init(uint8_t pin,  bool isWeelCounterEnabled, uint16_t arrB[], uint16_t arrF[]);
    uint16_t zeroPulse;
    uint8_t getTicks();
    void setForward(uint16_t arr[]); 
    void setBackward(uint16_t arr[]);
    void setSpeed(int8_t value);
    int8_t getSpeed();
    uint8_t getRealSpeed();
    void drive();
    void stop();
    void start();
    void calcRealSpeed();
  private:
    PWMServo motor;
    bool isTickCounter;
    uint8_t currentSpeed;
    int8_t settedSpeed;
    uint16_t forwardSpeeds[10];
    uint16_t backwardSpeeds[10];
    bool disabled;
    uint16_t lastTime;
};
#endif