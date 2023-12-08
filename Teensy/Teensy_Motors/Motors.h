#ifndef _MOTORS_H
#define _MOTORS_H

#include <Arduino.h>
#include <Print.h>
#include <Servo.h>

#endif


#define PIN_MOTOR1 5
#define PIN_MOTOR2 2
#define PIN_MOTOR3 3
#define PIN_MOTOR4 4

#define min 1000
#define max 2000

class Motors{
    public:

    Servo m1;
    Servo m2;
    Servo m3;
    Servo m4;

    Servo *m_all[4] = {&m1, &m2, &m3, &m4};

    int m1_offset = 1048-min; // start values in microseconds.
    int m2_offset = 1053-min; // start values in microseconds.
    int m3_offset = 1048-min; // start values in microseconds.
    int m4_offset = 1051-min; // start values in microseconds.

    int *m_offsets[4] = {&m1_offset, &m2_offset, &m3_offset, &m4_offset};

    void attach();
    void calibrate();
    void test_calibration(int);
    void find_offsets(int motor, int start, int upper, int lower);
    void set_micro(int);
    void set_micro_scaled(int);

};






