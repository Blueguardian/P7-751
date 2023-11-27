#include "GY88_sensors.h"


GY88_Sensors sensors;
bool blinkState = false;


void setup() {
	Serial.begin(115200);
    Serial.setTimeout(0);
	
    // configure Arduino LED pin for output
    pinMode(LED_BUILTIN, OUTPUT);
    
	while(!sensors.begin()){ delay(2000); }

    sensors.barom.sea_lvl_press = 1011; // hPa checked on the web.

    sensors.calibrate();
    digitalWrite(LED_BUILTIN, blinkState); // Led on indicating that the sensors are already calibrated.
}

void loop() {
    while (!Serial){ delay(10); } // Checking if anyone is still connected.
	
    sensors.update();
	sensors.print();

    blinkState = !blinkState;

    digitalWrite(LED_BUILTIN, blinkState);
    delay(100);
}








