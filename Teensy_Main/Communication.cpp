#include "Communication.h"


Communication::Communication() {
    Serial.begin(115200);
    Serial.setTimeout(0);
}

void Communication::serial_com_setup(){
    Serial.begin(115200);
    Serial.setTimeout(0);
}

void Communication::await_for_connection(){
    while (!Serial){ 
        data_sent=false; 
        delay(10); 
        
    } // Checking if anyone is still connected.
}

void Communication::send_receive(Sensors* sensors){
    
    if(!data_sent){
        sensors->print_xml();
        data_sent = true;
    }else if(Serial.available()){
        command = Serial.readString(); // python doesn't add EOL
        data_sent = false;
    }

}
