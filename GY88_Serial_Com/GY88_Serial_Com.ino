#include "Teensy.h"

Teensy teensy;

bool blinkState = true;

void setup() {
    teensy.serial_com_setup();
	
    pinMode(LED_BUILTIN, OUTPUT);   // configure Arduino LED pin for output
    
	while(!teensy.begin_sensors()){ delay(2000); } // Wait for all the sensors to be ready.

    teensy.calibrate();
    digitalWrite(LED_BUILTIN, HIGH); // Led on indicating that the sensors are already calibrated.
}


void loop() {
    teensy.await_for_connection(); // Checking if anyone is connected.
	
    teensy.update_sensors();

    teensy.send_receive(); // sends sensor data and reads commands through serial port.

    // blink LED to indicate activity
    if(String("command: blink_enabled").equals(teensy.command))
        blinkState = !blinkState;

    digitalWrite(LED_BUILTIN, blinkState);

}








