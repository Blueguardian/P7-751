#include "GY88_sensors.h"

GY88_Sensors sensors;

bool blinkState = false;

void setup() {

	Serial.begin(38400); // 38400
	
	while (!Serial){ delay(10); }

	while(!sensors.begin()){ delay(2000); }

    sensors.calibrate();

    // configure Arduino LED pin for output
    pinMode(LED_BUILTIN, OUTPUT);
}


void loop() {
	sensors.update();
	sensors.print();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
}
