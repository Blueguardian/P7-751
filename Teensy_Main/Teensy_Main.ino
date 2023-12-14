#include "Motors.h"
#include "Controller.h"
#include "Communication.h"

Sensors sensors;
Communication communication;
Motors motors;
PID_Controller controller;


void setup() {
    pinMode(LED_BUILTIN, OUTPUT);           // configure Teensy LED pin as output.
    
    motors.attach();                        // Initialise motors.
    motors.set_pwm(0);                      // Stop motors.

    communication.serial_com_setup();       // Initialise Serial Comm.
	
    sensors.begin_sensors();                // Initialise Sensors.
    sensors.calibrate();                    // Calibrate Sensors.

    controller.begin();                     // Initialse PID controller.

    digitalWrite(LED_BUILTIN, HIGH);        // Led on indicating that the drone is ready.

}

void parse_xml(String command, double *states, bool &start, double &alt){

    float readings[11];
    String n;
    for(int i=1; i<=11; i++){
        command = command.substring(command.indexOf("num=\""+String(i)+"\">")+String("num=\""+String(i)+"\">").length());
        n = command.substring(0,command.indexOf("<"));
        command = command.substring(command.indexOf("<")+1);
        readings[i-1] = n.toFloat();
    }

    start = bool(readings[0]);
    alt = readings[1];

    for(int i=0; i<9; i++){
        states[i] = readings[i+2];
    }

    //'<Vector type="np.float64" origin="PI">np.float64<INDEX num="1">0.2</INDEX><INDEX num="2">0.2</INDEX><INDEX num="3">0.2</INDEX><INDEX num="4">0.2</INDEX><INDEX num="5">0.2</INDEX><INDEX num="6">0.2</INDEX><INDEX num="7">0.2</INDEX><INDEX num="8">0.2</INDEX><INDEX num="9">0.2</INDEX><INDEX num="10">0.2</INDEX><INDEX num="11">0.2</INDEX></Vector>'


}




void loop() {

    while (!Serial){ 
        communication.data_sent=false; 
        motors.set_pwm(0);
        delay(10); 
        
    }  // Checking if anyone is connected.
	
    double states[9];
    double alt=0;
    bool start = false;

    sensors.update_sensors();                   // Update sensor readings.
    communication.send_receive(&sensors);       // Sends sensor data and reads commands through serial port.


    if(communication.command){
        parse_xml(communication.command, states, start, alt);
        // Serial.println(String(start) + " " + String(alt));
        if(start){
            digitalWrite(LED_BUILTIN, HIGH);
        }else{
            digitalWrite(LED_BUILTIN, LOW);
        }
    }



    if(start){
        controller.alt = alt;
        controller.update(states[0], states[1], states[2], states[6], states[7], states[8]); // EKF_x, EKF_y, EKF_z, EKF_pitch, EKF_yaw, EKF_roll

        for(int i=0; i<4; i++){
            motors.set_pwm(controller.pwm_matrix[i],i);
        }

    }else{
        motors.set_pwm(0);
    }

    
    //digitalWrite(LED_BUILTIN, blinkState);

}








