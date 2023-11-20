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

class GY88_Sensors{
	private:
		MPU6050 mpu;
		Adafruit_HMC5883_Unified hmc;
		Adafruit_BMP085 bmp;

		sensors_event_t temp_mag;
        int16_t raw_ax, raw_ay, raw_az;
        int16_t raw_gx, raw_gy, raw_gz;

	public:
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
			float press;		    // Pa
			float alt;			    // m
			float press_sea_lvl;    // m
			float real_alt;		    // m
		} barom;

		GY88_Sensors(float declination_angle = 0.0698132);
		bool begin();
		void update(); // Read sensors.
		void print();
        void get_sensors_ranges();
        void set_sensors_ranges(int, int);
        void set_accel_offsets(int, int, int);
        void set_gyro_offsets(int, int, int);
        void get_offsets();
        void calibrate();
};







