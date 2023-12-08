#include "Motors.h"

#define PCB_LED 6

Motors motors;


void setup(){
    Serial.begin(9600);
    
    motors.attach();

    //motors.calibrate(); // If you run the calibration, you will have to find again the offsets for each motor!.

    motors.set_micro_scaled(0); // Stop the motors.

}

void loop(){
    motors.test_calibration(20); // loop from 0 to 10 and from 10 to 0

    //motors.set_micro_scaled(1);


}





