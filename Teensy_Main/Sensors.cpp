#include "Sensors.h"


Sensors::Sensors(float declination_angle, float sea_lvl_press) {
    magnet1.declination_angle = declination_angle;   // in deg.
    barom1.sea_lvl_press = sea_lvl_press;            // in hPa
    barom2.sea_lvl_press = sea_lvl_press;            // in hPa
}

bool Sensors::begin_sensors() {
    Wire.begin();
    Wire1.begin();

    mpu1 = new MPU6050(0x68, &Wire);
    mpu2 = new MPU6050(0x68, &Wire1);

	mpu1->initialize();
	mpu2->initialize();
    
    if (!mpu1->testConnection())   { Serial.println("Failed to find MPU6050 chip 1");   return false; }
    if (!mpu2->testConnection())   { Serial.println("Failed to find MPU6050 chip 2");    }
    if (!hmc1.begin())             { Serial.println("Failed to find HMC5883 chip");      }
    if (!bmp1.begin(0x77, &Wire))  { Serial.println("Failed to find BMP085  chip");      }
    if (!bmp2.begin(0x76, &Wire1)) { Serial.println("Failed to find BMP085  chip");      }


    return true;
}

void Sensors::update_sensors() {
    short gyro_range = 250 * pow(2, mpu1->getFullScaleAccelRange()); // 250, 500, 1000, 2000.
    short acc_range  = 2   * pow(2, mpu1->getFullScaleAccelRange()); // 2, 4, 8, 16.

    mpu1->getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    accel1.x = map((float)raw_ax, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;               // scaling from raw measures to real values.
    accel1.y = map((float)raw_ay, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;      
    accel1.z = map((float)raw_az, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;     // same + removing gravity.
    gyro1.x  = map((float)raw_gx, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s
    gyro1.y  = map((float)raw_gy, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s
    gyro1.z  = map((float)raw_gz, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s

    mpu2->getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    accel2.x = map((float)raw_ax, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;               // scaling from raw measures to real values.
    accel2.y = map((float)raw_ay, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;      
    accel2.z = map((float)raw_az, -32768, 32757, -acc_range,  acc_range)  * GRAVITY;     // same + removing gravity.
    gyro2.x  = map((float)raw_gx, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s
    gyro2.y  = map((float)raw_gy, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s
    gyro2.z  = map((float)raw_gz, -32768, 32757, -gyro_range, gyro_range) * M_PI / 180; // rad/s

    hmc1.getEvent(&raw_mag);
    magnet1.x = raw_mag.magnetic.x;
    magnet1.y = raw_mag.magnetic.y; 
    magnet1.z = raw_mag.magnetic.z;

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(magnet1.y, magnet1.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    heading += magnet1.declination_angle;
    
    if (heading < 0) {heading += 2 * PI;}       // Correct for when signs are reversed.
    if (heading > 2 * PI) {heading -= 2 * PI;}  // Check for wrap due to addition of declination.
        
    magnet1.heading = heading * 180 / M_PI;     // Convert radians to degrees for readability.


    barom1.alt = bmp1.readAltitude(barom1.sea_lvl_press) - barom1.alt_offset;  // sea lvl press in Pa.
    barom2.alt = bmp2.readAltitude(barom2.sea_lvl_press) - barom2.alt_offset;  // sea lvl press in Pa.

    time_stamp = millis();
}

void Sensors::print() {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("(Accel_1) \tX: " + String(accel1.x,3));
    Serial.print(" \tY: "         + String(accel1.y,3));
    Serial.println(" \tZ: "       + String(accel1.z,3) + " m/s^2  ");
    Serial.print("(Accel_2) \tX: " + String(accel2.x,3));
    Serial.print(" \tY: "         + String(accel2.y,3));
    Serial.println(" \tZ: "       + String(accel2.z,3) + " m/s^2 \n");

    /* Display the results (rotation is measured in rad/s) */
    Serial.print("(Gyro_1)  \tX: "   + String(gyro1.x,3));
    Serial.print(" \tY: "           + String(gyro1.y,3));
    Serial.println(" \tZ: "         + String(gyro1.z,3) + " rad/s ");
    Serial.print("(Gyro_2)  \tX: "   + String(gyro2.x,3));
    Serial.print(" \tY: "           + String(gyro2.y,3));
    Serial.println(" \tZ: "         + String(gyro2.z,3) + " rad/s \n");

    /* Display the results*/
    Serial.print("(Barom_1) \tAlt: "     + String(barom1.alt) + " m");
    Serial.println(" \tSea_lvl_press: " + String(barom1.sea_lvl_press) + " Pa");
    Serial.print("(Barom_2) \tAlt: "     + String(barom2.alt) + " m");
    Serial.println(" \tSea_lvl_press: " + String(barom2.sea_lvl_press) + " Pa");

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("(Magnet_1) \tX: "  + String(magnet1.x));
    Serial.print(" \tY: "           + String(magnet1.y));
    Serial.print(" \tZ: "           + String(magnet1.z) + " uT ");
    Serial.println("\tHeading: "    + String(magnet1.heading) + "º\n");
}

void Sensors::get_sensors_ranges() {
    //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    Serial.print("Accelerometer range set to: ");
    switch (mpu1->getFullScaleAccelRange()) {
        case MPU6050_ACCEL_FS_2:
        Serial.println("+-2G");
        break;
        case MPU6050_ACCEL_FS_4:
        Serial.println("+-4G");
        break;
        case MPU6050_ACCEL_FS_8:
        Serial.println("+-8G");
        break;
        case MPU6050_ACCEL_FS_16:
        Serial.println("+-16G");
        break;
    }
    
    //mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    Serial.print("Gyro range set to: ");
    switch (mpu1->getFullScaleGyroRange()) {
        case MPU6050_GYRO_FS_250:
        Serial.println("+- 250 deg/s");
        break;
        case MPU6050_GYRO_FS_500:
        Serial.println("+- 500 deg/s");
        break;
        case MPU6050_GYRO_FS_1000:
        Serial.println("+- 1000 deg/s");
        break;
        case MPU6050_GYRO_FS_2000:
        Serial.println("+- 2000 deg/s");
        break;
    }

    Serial.println();
}

void Sensors::set_sensors_ranges(int ACCEL_FS, int GYRO_FS) {
    mpu1->setFullScaleAccelRange(ACCEL_FS);
    mpu2->setFullScaleAccelRange(ACCEL_FS);
    Serial.print("Accelerometer range set to: ");
    switch (mpu1->getFullScaleAccelRange()) {
        case MPU6050_ACCEL_FS_2:
        Serial.println("+-2G");
        break;
        case MPU6050_ACCEL_FS_4:
        Serial.println("+-4G");
        break;
        case MPU6050_ACCEL_FS_8:
        Serial.println("+-8G");
        break;
        case MPU6050_ACCEL_FS_16:
        Serial.println("+-16G");
        break;
    }
    
    mpu1->setFullScaleGyroRange(GYRO_FS);
    mpu2->setFullScaleGyroRange(GYRO_FS);
    Serial.print("Gyro range set to: ");
    switch (mpu1->getFullScaleGyroRange()) {
        case MPU6050_GYRO_FS_250:
        Serial.println("+- 250 deg/s");
        break;
        case MPU6050_GYRO_FS_500:
        Serial.println("+- 500 deg/s");
        break;
        case MPU6050_GYRO_FS_1000:
        Serial.println("+- 1000 deg/s");
        break;
        case MPU6050_GYRO_FS_2000:
        Serial.println("+- 2000 deg/s");
        break;
    }

    Serial.println();
}

void Sensors::set_mpu1_offsets(int16_t *offsets){
    mpu1->setXAccelOffset(offsets[0]);
    mpu1->setYAccelOffset(offsets[1]);
    mpu1->setZAccelOffset(offsets[2]);
    mpu1->setXGyroOffset(offsets[3]);
    mpu1->setYGyroOffset(offsets[4]);
    mpu1->setZGyroOffset(offsets[5]);
}

void Sensors::set_mpu2_offsets(int16_t *offsets){
    mpu2->setXAccelOffset(offsets[0]);
    mpu2->setYAccelOffset(offsets[1]);
    mpu2->setZAccelOffset(offsets[2]);
    mpu2->setXGyroOffset(offsets[3]);
    mpu2->setYGyroOffset(offsets[4]);
    mpu2->setZGyroOffset(offsets[5]);
}

void Sensors::get_offsets(){
    int ax = mpu1->getXAccelOffset();
    int ay = mpu1->getYAccelOffset();
    int az = mpu1->getZAccelOffset();
    int gx = mpu1->getXGyroOffset();
    int gy = mpu1->getYGyroOffset();
    int gz = mpu1->getZGyroOffset();

    Serial.println("Accel1 offsets: " + String(ax) + " , " + String(ay) + " , " + String(az));
    Serial.println("Gyro1 offsets: "  + String(gx) + " , " + String(gy) + " , " + String(gz));
    Serial.println("Barom1 offset: "  + String(barom1.alt_offset));

    ax = mpu2->getXAccelOffset();
    ay = mpu2->getYAccelOffset();
    az = mpu2->getZAccelOffset();
    gx = mpu2->getXGyroOffset();
    gy = mpu2->getYGyroOffset();
    gz = mpu2->getZGyroOffset();

    Serial.println("Accel2 offsets: " + String(ax) + " , " + String(ay) + " , " + String(az));
    Serial.println("Gyro2 offsets: "  + String(gx) + " , " + String(gy) + " , " + String(gz));
    Serial.println("Barom2 offset: "  + String(barom2.alt_offset));
}

void Sensors::calibrate(){

    get_offsets();
    mpu1->CalibrateAccel(10); // calculates the optimal offsets for the acceleration. so that x,y,z are (0g, 0g, 1g).
    mpu1->CalibrateGyro(10);  // Calculates the optimal offsets for the gyroscopes, so that x,y,z are (0 rad/s, 0 rad/s, 0 rad/s).
    mpu2->CalibrateAccel(10); // calculates the optimal offsets for the acceleration. so that x,y,z are (0g, 0g, 1g).
    mpu2->CalibrateGyro(10);  // Calculates the optimal offsets for the gyroscopes, so that x,y,z are (0 rad/s, 0 rad/s, 0 rad/s).

    int16_t *offsets1 = mpu1->GetActiveOffsets(); // Getting these optimal offsets we just calculated.
    set_mpu1_offsets(offsets1);

    int16_t *offsets2 = mpu2->GetActiveOffsets(); // Getting these optimal offsets we just calculated.
    set_mpu2_offsets(offsets2);

    barom1.alt_offset = 0;
    barom2.alt_offset = 0;
    for(int i=0; i<50; i++){ 
        barom1.alt_offset += bmp1.readAltitude(barom1.sea_lvl_press);
        barom2.alt_offset += bmp2.readAltitude(barom2.sea_lvl_press);
    }
    barom1.alt_offset /= 50;
    barom2.alt_offset /= 50;
      
    get_offsets();

    Serial.println("MPU calibrated!.");
}

String Sensors::get_xml(){
    float m[3][6] = {
                        {accel1.x,      accel1.y,       accel1.z,       gyro1.x,        gyro1.y,        gyro1.z   },
                        {accel2.x,      accel2.y,       accel2.z,       gyro2.x,        gyro2.y,        gyro2.z   },
                        {magnet1.x,     magnet1.y,      magnet1.z,      barom1.alt,     barom2.alt,     time_stamp}
                    };

    String str = "<Matrix type=\"np.float64\" origin=\"IMU\">";
    for(int i=0; i<3; i++){
        str+= "<ROW num=\""+String(i)+"\">";
        
        for(int j=0;j<6;j++){
            str+="<INDEX num=\""+String(j+1)+"\">"+String(m[i][j], 6)+"</INDEX>";
        }

        str+="</ROW>";
    }
    str+="</Matrix>";

    return str;
}

void Sensors::print_xml(){

    Serial.println(get_xml());
    
}





