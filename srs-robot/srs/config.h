#ifndef config_h
#define config_h

// #define PID_MODE

// Debug mode
#define DEBUG_MODE


// distanse (analog sensors SHARP)
#define ADC_BIT_SIZE 10
#define MEDIAN_SIZE 3
#define NUMBER_OF_SENSORS 6


#define SEN_R45 2
#define SEN_L0 3
#define SEN_R0 0
#define SEN_L45 1
#define SEN_L90 4
#define SEN_R90 5


// SERIAL
#define SERIAL_SPEED 115200
#define UI_PARAMS_COUNT 7


// MOTOR
#define FORWARD_SPEED {1480, 1520, 1570, 1600, 1630, 1730, 2000, 2000, 2000, 2500}
#define BACKWARD_SPEED {1420, 1380, 1300, 1250, 1200, 1150, 1100, 1000, 800, 500}
#define MOTOR_PIN 10


// SERVO
#define SERVO_PIN 9
#define SERVO_OFFSET 90
#define SERVO_MAX_ANGLE 89
#define SERVO_MAX_MICROS 2040
#define SERVO_MIN_MICROS 950


#define ENABLE_WIRE

#endif