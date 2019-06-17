#include <Arduino.h>
#include <Motor.hpp>

void setup() {
    // put your setup code here, to run once:
}

Motor m1;
uint16_t forwardSpeeds[10]  = {1550,2000,2000,2000,2000,2000,2000,2000,2000,2000};
uint16_t backwardSpeeds[10] = {1450,1450,1450,1450,1450,1450,1450,1450,1450,1000};

uint16_t period = 100;
uint16_t lastTime = 0;

void loop() {
    m1.init(10, true, backwardSpeeds, forwardSpeeds);
    delay(4000);
    m1.setSpeed(2);
    while(1) {
        m1.drive();
        if (lastTime + period > millis()) {
            lastTime = millis();
            m1.calcRealSpeed();
            Serial.println(m1.getRealSpeed());
        }
    }
    // put your main code here, to run repeatedly:
}