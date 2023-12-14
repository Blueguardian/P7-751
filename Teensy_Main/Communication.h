#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "Arduino.h"
#include "Print.h"
#include "Sensors.h"

#endif


class Communication{
	public:
        bool data_sent = false;
        String command = "";

		Communication();
        void serial_com_setup();
        void await_for_connection();
        void send_receive(Sensors*);
};




