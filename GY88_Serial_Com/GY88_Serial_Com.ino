#include "GY88_sensors.h"

GY88_Sensors sensors;

bool blinkState = true;
bool data_sent = false;
String line = String("");

void setup() {
	Serial.begin(115200);
    Serial.setTimeout(0);
	
    // configure Arduino LED pin for output
    pinMode(LED_BUILTIN, OUTPUT);
    
	while(!sensors.begin()){ delay(2000); }

    sensors.calibrate();
    digitalWrite(LED_BUILTIN, blinkState); // Led on indicating that the sensors are already calibrated.
}



void loop() {
    while (!Serial){ data_sent=false; delay(10); } // Checking if anyone is still connected.
	
    sensors.update();
	//sensors.print();

    if(!data_sent){
        sensors.print_xml();
        data_sent = true;
    }else if(Serial.available()){
        line = String(Serial.readString()); // python doesn't add EOL
        data_sent = false;
    }


    // blink LED to indicate activity
    if(String("command: blink_enabled").equals(line))
        blinkState = !blinkState;

    digitalWrite(LED_BUILTIN, blinkState);
}








