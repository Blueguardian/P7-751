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

#define MIN 1000
#define MAX 2000

class Motors{
    private:
        Servo m1;
        Servo m2;
        Servo m3;
        Servo m4;
        Servo *m_all[4] = {&m1, &m2, &m3, &m4};

        int m1_offset = 1048-MIN; // start values in microseconds.
        int m2_offset = 1053-MIN; // start values in microseconds.
        int m3_offset = 1048-MIN; // start values in microseconds.
        int m4_offset = 1051-MIN; // start values in microseconds.
        int *m_offsets[4] = {&m1_offset, &m2_offset, &m3_offset, &m4_offset};


        void set_micro(int);
    public:

        void attach();
        void calibrate();                           // calibrate motors (set minimun and maximum).
        void test_calibration(int);                 // test calibrarion of motors incrementing pwm values.
        void find_offsets(int, int, int, int);      // find offsets of each motor.
        void set_pwm(int);                          // set pwm value for all the motors.
        void set_pwm(int, int);                     // set pwm value for a specific motor [0-3].


};






