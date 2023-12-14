#ifndef _READ_SENSORS_H
#define _READ_SENSORS_H

#include <MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>
#include "I2Cdev.h"
#include "Arduino.h"
#include "Print.h"


#endif

#define GRAVITY 9.80665



class Sensors{
	private:
		MPU6050 *mpu1;
		MPU6050 *mpu2;
        
        Adafruit_BMP085 bmp1;
        Adafruit_BMP085 bmp2;

		Adafruit_HMC5883_Unified hmc1;

		sensors_event_t raw_mag;
        int16_t raw_ax, raw_ay, raw_az;
        int16_t raw_gx, raw_gy, raw_gz;

        bool data_sent = false;
        float time_stamp = 0;

	public:
        String command = "";

		struct{
			float x; float y; float z;
		} accel1, accel2, gyro1, gyro2;
        
		struct{
			float x; float y; float z;
			float heading;
			float declination_angle;
		} magnet1;

		struct{
			float alt;			    // m
			float sea_lvl_press;    // hPa
            float alt_offset = 0;
		} barom1, barom2;

		Sensors(float declination_angle = 0.0698132, float sea_lvl_press = 1003.1 *100); // sea_lvl_press in Pa.
		bool begin_sensors();
		void update_sensors(); // Read sensors.
		void print();
        void get_sensors_ranges();
        void set_sensors_ranges(int, int);
        void set_mpu1_offsets(int16_t *);
        void set_mpu2_offsets(int16_t *);
        void get_offsets();
        void calibrate();
        String get_xml();
        void print_xml();
};




