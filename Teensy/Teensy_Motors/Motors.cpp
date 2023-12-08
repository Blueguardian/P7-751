#include "Motors.h"

void Motors::attach(){
    // Setting motors pins and minimum, maximum pulse width.
	m1.attach(PIN_MOTOR1, min, max);
	m2.attach(PIN_MOTOR2, min, max);
	m3.attach(PIN_MOTOR3, min, max);
	m4.attach(PIN_MOTOR4, min, max);
}

void Motors::calibrate(){
    Serial.println("seting maximum:");
    for(int i = 0; i<200; i++){
        set_micro(max);
        delay(10);
    }

    Serial.println("seting minimum:");
    for(int i = 0; i<200; i++){
        set_micro(min);
        delay(10);
    }

    Serial.println("Calibrated!");
}

void Motors::test_calibration(int range=10){
    digitalWrite(LED_BUILTIN, HIGH);

    for(int i = 0; i<=range ; i++){
        set_micro_scaled(i);
        Serial.println(String(i));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
	}

    delay(1000);

    for(int i = range; i>=0 ; i--){
        set_micro_scaled(i);
        Serial.println(String(i));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
	}


    delay(2000);
}

void Motors::find_offsets(int motor, int start=1045, int upper=10, int lower=10){


    for(int i = 0; i<=upper ; i++){
        int val = start + i;

        m_all[motor]->writeMicroseconds(val);
        Serial.println(String(i) + " " + String(val));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
	}

    delay(1000);

    for(int i = 0; i<=lower ; i++){
        //int val = map(i, 0, 100, min, max);
        int val = start - i;

        m_all[motor]->writeMicroseconds(val);
        Serial.println(String(i) + " " + String(val));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
	}


    delay(2000);
}

void Motors::set_micro(int val){
    for(int i=0; i<4; i++){
        m_all[i]->writeMicroseconds(val);
    }
}

void Motors::set_micro_scaled(int value){ // Sets speed values from 0 to 250.
    
    for(int i=0; i<4; i++){
        int val = min;

        if(value){
            val = map(value, 1, 250, min + *m_offsets[i], max + *m_offsets[i]);
        }

        m_all[i]->writeMicroseconds(val);

    }

}