#include "Motor.hpp"
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define SPEED_LENGTH 10


volatile uint8_t tickCounter = 0;
bool isTickCounter = false;


Motor::Motor(){
}

void tick() {
  tickCounter++;
}


uint8_t Motor::getRealSpeed() {
  return this->currentSpeed;
}


void Motor::stop() {
  this->disabled = true;
}


void Motor::start() {
  this->disabled = false;
}


void Motor::init(uint8_t pin, bool isWeelCounterEnabled, uint16_t arrB[], uint16_t arrF[]) {
  this->setForward(arrF);
  this->setBackward(arrB);
  this->zeroPulse = this->forwardSpeeds[0];
  this->isTickCounter = false;
  this->motor = PWMServo();
  this->motor.attach(pin, arrB[SPEED_LENGTH-1], arrF[SPEED_LENGTH-1]);
  this->motor.writeMicroseconds(this->zeroPulse); 
  this->settedSpeed = 0;
  this->disabled = false;

  if (isWeelCounterEnabled == false && isTickCounter == false) {
    attachInterrupt(0, tick, FALLING);
    isTickCounter = true;
    this->isTickCounter = true;
  }

  delay(4000);
}


void Motor::drive() {
  if (this->disabled) {
    this->settedSpeed = 0;
  }

  if (this->settedSpeed > 0) {
    this->settedSpeed = constrain(this->settedSpeed,0,sizeof(this->forwardSpeeds)/sizeof(uint16_t)-1);
    this->motor.writeMicroseconds(this->forwardSpeeds[this->settedSpeed]);
  } else if (this->settedSpeed == 0) {
    this->motor.writeMicroseconds(this->zeroPulse);
  } else if (this->settedSpeed < 0) {
    this->settedSpeed = constrain(this->settedSpeed * -1,0,sizeof(this->backwardSpeeds)/sizeof(uint16_t)-1);
    this->motor.writeMicroseconds(this->backwardSpeeds[this->settedSpeed]);
    this->settedSpeed = this->settedSpeed * -1;
  }
}


void Motor::setSpeed(int8_t value) {
  this->settedSpeed = value;
}


int8_t Motor::getSpeed() {
  return this->settedSpeed;
}


void Motor::setForward(uint16_t arr[]) {
  this->forwardSpeeds[0] = arr[0];
  this->forwardSpeeds[1] = arr[1];
  this->forwardSpeeds[2] = arr[2];
  this->forwardSpeeds[3] = arr[3];
  this->forwardSpeeds[4] = arr[4];
  this->forwardSpeeds[5] = arr[5];
  this->forwardSpeeds[6] = arr[6];
  this->forwardSpeeds[7] = arr[7];
  this->forwardSpeeds[8] = arr[8];
  this->forwardSpeeds[9] = arr[9];
}


void Motor::setBackward(uint16_t arr[]) {
    this->backwardSpeeds[0] = arr[0];
    this->backwardSpeeds[1] = arr[1];
    this->backwardSpeeds[2] = arr[2];
    this->backwardSpeeds[3] = arr[3];
    this->backwardSpeeds[4] = arr[4];
    this->backwardSpeeds[5] = arr[5];
    this->backwardSpeeds[6] = arr[6];
    this->backwardSpeeds[7] = arr[7];
    this->backwardSpeeds[8] = arr[8];
    this->backwardSpeeds[9] = arr[9];
}


void Motor::calcRealSpeed() {
  if (this->isTickCounter) {
    this->currentSpeed = this->getTicks();
  }
}


uint8_t Motor::getTicks() {
  uint16_t currenttime = millis();
  uint16_t period = currenttime - Motor::lastTime;
  Motor::lastTime = currenttime;
  static uint8_t value = 0; 
  cli();
  value = tickCounter;
  tickCounter = 0;
  sei();

  return (value * 100) / period;


  // static uint8_t wheelCountArray[] = {0,0,0,0,0};
  // static uint8_t arIndex = 0;
  // static uint8_t speedSumm = 0;
  // speedSumm -= wheelCountArray[arIndex];
  // wheelCountArray[arIndex] = value;
  // arIndex++;
  // arIndex %= sizeof(wheelCountArray);
  // speedSumm += value;


  // return speedSumm / sizeof(wheelCountArray);
}





























/*
 * Calculate real speed *
 * big gear 86t 12 tics of wheel counter
 * small 15t
 * big 34t
 * radius of wheel = 30 mm
 * 15/34 * 60 * 3.14 is for 12 tics and it is 83
 * 1 tic 7 mm per period
 */
/*
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor.h"
#include <PID_v1.h>

const int16_t Motor::speedForward[] = {
  0,
  1,
  2,
  3,
  5,
  7,
  8,
  11,
  15,
  16,
  19,
  24,
  27,
  31
};

// int16_t Motor::forwardMax;
int16_t Motor::forwardMin;
int16_t Motor::speedZero;
int16_t Motor::backMin;
int16_t Motor::backMax;
int16_t Motor::ledPin;
int16_t Motor::maxAngle;
int16_t Motor::servoOfset;

PWMServo Motor::motor;
PWMServo Motor::wheelServoR;

int16_t Motor::speed = 0; // setted robot speed in tiks in MOTOR_PERIOD ms
int16_t Motor::angle = 0;

volatile uint8_t Motor::realRobotSpeed = 0; // real robot speed in tiks in MOTOR_PERIOD ms
volatile uint8_t Motor::wheelCount = 0;

uint8_t Motor::getRealSpeed() {
  return Motor::realRobotSpeed;
}

uint8_t Motor::getAngle() {
  return Motor::angle;
}

uint8_t Motor::getSpeed() {
  return Motor::speed;
}

void Motor::setSpeed(int16_t value) {
  Motor::speed = value;
}

void Motor::drive() {
  Motor::realRobotSpeed = Motor::getTicks();
  int16_t value = Motor::speed;
  if(value < 0) {
    Motor::speed = value = constrain(value, Motor::backMax, Motor::backMin);
    motor.write(value + Motor::backMin);
  } else {
    if(value == 0) {
      motor.write(Motor::speedZero);
    }
    else {
      #define MAX_SPEED_PARAM sizeof(speedForward)/sizeof( typeof(speedForward) ) - 1
      if (value >  MAX_SPEED_PARAM ) {
        value = MAX_SPEED_PARAM;
      }
      Motor::speed = speedForward[value];
      motor.write(Motor::forwardMin + speedForward[value]);
    }
  }

}

void Motor::motorTick() {
  Motor::wheelCount++;
}

uint8_t Motor::getTicks() {
  static uint8_t wheelCountArray[] = {0,0,0,0,0};
  static uint8_t arIndex = 0;
  static uint16_t speedSumm = 0;
  cli();
  uint8_t value =  Motor::wheelCount;
  Motor::wheelCount = 0;
  sei();
  speedSumm -= wheelCountArray[arIndex];
  wheelCountArray[arIndex] = value;
  arIndex++;
  arIndex %= sizeof(wheelCountArray);
  speedSumm += value;
  return speedSumm;
}


void Motor::setAngle (int16_t value) {
  Motor::angle = constrain(value, -Motor::maxAngle, Motor::maxAngle);
  wheelServoR.write(Motor::servoOfset + Motor::angle);
}

void Motor::ledSwitch(boolean value) {
  if (value == HIGH) digitalWrite(Motor::ledPin, HIGH);
  else digitalWrite(Motor::ledPin, LOW);
}

void Motor::toggleLed() {
  static boolean st = HIGH;
  ledSwitch(st);
  st = !st;
}

void Motor::init(
    uint8_t motorPinVal,
    int16_t forwardMinVal,
    int16_t speedZeroVal,
    int16_t backMinVal,
    int16_t backMaxVal,
    int16_t ledPinVal,
    uint8_t servoPinVal,
    int16_t maxAngleVal,
    int16_t servoOfsetVal
  ) {
  Motor::forwardMin = forwardMinVal;
  Motor::speedZero = speedZeroVal;
  Motor::backMin = backMinVal;
  Motor::backMax = backMaxVal;
  Motor::ledPin = ledPinVal;
  Motor::maxAngle = maxAngleVal;
  Motor::servoOfset = servoOfsetVal;
  

  // init moror
  motor.attach(motorPinVal); // 10
  motor.write(Motor::speedZero);
  attachInterrupt(0, &Motor::motorTick, FALLING);`
  delay(4000);

  // initialize the LED pin as an output:
  pinMode(ledPinVal, OUTPUT);
  Motor::ledSwitch(LOW);

  // init servo and set it to midle
  wheelServoR.attach(servoPinVal,950,2040);  // 9
  Motor::setAngle(0);

}
*/