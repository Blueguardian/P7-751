#ifndef _READ_SENSORS_H
#define _READ_SENSORS_H

#include <MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
#include "I2Cdev.h"
#include "Arduino.h"
#include "Print.h"

#endif

#define GRAVITY 9.80665

class Teensy{
	private:
		MPU6050 mpu;
		Adafruit_HMC5883_Unified hmc;
        Adafruit_BMP085_Unified bmp;

		sensors_event_t temp_mag;
        int16_t raw_ax, raw_ay, raw_az;
        int16_t raw_gx, raw_gy, raw_gz;

        bool data_sent = false;
        unsigned long time_stamp = 0;

	public:
        String command = "";

		struct{
			float x; float y; float z;
            float sqrt;
		} accel;
        
        struct{
			float x; float y; float z;
		} gyro;

		struct{
			float x; float y; float z;
			float heading;
			float declination_angle;
		} magnet;

		struct{
			float temp; 		    // ºC
			float press;		    // hPa
			float alt;			    // m
			float sea_lvl_press;    // hPa
            float alt_offset = 0;
		} barom;

		Teensy(float declination_angle = 0.0698132, float sea_lvl_press = SENSORS_PRESSURE_SEALEVELHPA);
		bool begin_sensors();
		void update_sensors(); // Read sensors.
		void print();
        void get_sensors_ranges();
        void set_sensors_ranges(int, int);
        void set_accel_offsets(int, int, int);
        void set_gyro_offsets(int, int, int);
        void get_offsets();
        void calibrate();
        String get_xml();
        void print_xml();

        void serial_com_setup();
        void await_for_connection();
        void send_receive();
};




