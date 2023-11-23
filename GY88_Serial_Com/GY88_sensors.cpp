#include "GY88_sensors.h"


GY88_Sensors::GY88_Sensors(float declination_angle, float sea_lvl_press) {
    magnet.declination_angle = declination_angle;   // in deg.
    barom.sea_lvl_press = sea_lvl_press;            // in hPa
}

bool GY88_Sensors::begin() {
    Wire.begin();

    mpu.initialize();

    if (!mpu.testConnection())  { Serial.println("Failed to find MPU6050 chip"); return false; }
    if (!hmc.begin())           { Serial.println("Failed to find HMC5883 chip"); return false; }
    //if (!bmp.begin())           { Serial.println("Failed to find BMP085  chip"); return false; }
    if (!bmp.begin(BMP085_MODE_ULTRALOWPOWER))  { Serial.println("Failed to find BMP085  chip"); return false; } // bmp faster but with bigger error.


    return true;
}

void GY88_Sensors::update() {
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    hmc.getEvent(&temp_mag);

    float gyro_factor  = 32768. / ( 250 * pow(2, mpu.getFullScaleGyroRange())  );
    float accel_factor = 32768. / ( 2   * pow(2, mpu.getFullScaleAccelRange()) );

    accel.x = raw_ax / accel_factor * GRAVITY;
    accel.y = raw_ay / accel_factor * GRAVITY;
    accel.z = raw_az / accel_factor * GRAVITY; 
    accel.sqrt =  sqrt(pow(accel.x, 2) + pow(accel.y, 2) + pow(accel.z, 2));

    gyro.x = (raw_gx / gyro_factor) * M_PI / 180; // rad/s
    gyro.y = (raw_gy / gyro_factor) * M_PI / 180;
    gyro.z = (raw_gz / gyro_factor) * M_PI / 180;

    magnet.x = temp_mag.magnetic.x;
    magnet.y = temp_mag.magnetic.y;
    magnet.z = temp_mag.magnetic.z;

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(magnet.y, magnet.x);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    heading += magnet.declination_angle;

    // Correct for when signs are reversed.
    if (heading < 0)
        heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
        heading -= 2 * PI;

    // Convert radians to degrees for readability.
    magnet.heading = heading * 180 / M_PI;

    bmp.getTemperature(&barom.temp);        // temp in deg.
    bmp.getPressure(&barom.press);          // press in kPa*10.
    barom.press /= 100;                    // press in hPa.
    barom.alt = bmp.pressureToAltitude(barom.sea_lvl_press, barom.press);   // Param: (sea level pressure in hPa, current pressure in hPa). altitude in m.
    //barom.sea_lvl_press = bmp.seaLevelForAltitude(barom.alt, barom.press); // Param: (current altitude (m), curent pressure (hPa)). Pressure at sea level in hPa.
}

void GY88_Sensors::print() {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("(Accel) \tX: " + String(accel.x));
    Serial.print(" \tY: " + String(accel.y));
    Serial.print(" \tZ: " + String(accel.z) + " m/s^2 ");
    Serial.println(" \tSqrt: " + String(accel.sqrt) + " m/s^2 ");

    /* Display the results (rotation is measured in rad/s) */
    Serial.print("(Gyro)  \tX: " + String(gyro.x));
    Serial.print(" \tY: " + String(gyro.y));
    Serial.println(" \tZ: " + String(gyro.z) + " rad/s ");
    Serial.println();

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("(Magnet) \tX: " + String(magnet.x));
    Serial.print(" \tY: " + String(magnet.y));
    Serial.print(" \tZ: " + String(magnet.z) + " uT ");
    Serial.println("\tHeading: " + String(magnet.heading) + "º");
    Serial.println();

    /* Display the results*/
    Serial.print("(Barom) \tTemp: " + String(barom.temp) + " ºC");
    Serial.print(" \tPress: " + String(barom.press) + " hPa");
    Serial.print(" \tAlt: " + String(barom.alt) + " m");
    Serial.println(" \tSea_lvl_press: " + String(barom.sea_lvl_press) + " hPa");
}

void GY88_Sensors::get_sensors_ranges() {
    //mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getFullScaleAccelRange()) {
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
    switch (mpu.getFullScaleGyroRange()) {
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

void GY88_Sensors::set_sensors_ranges(int ACCEL_FS, int GYRO_FS) {
    mpu.setFullScaleAccelRange(ACCEL_FS);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getFullScaleAccelRange()) {
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
    
    mpu.setFullScaleGyroRange(GYRO_FS);
    Serial.print("Gyro range set to: ");
    switch (mpu.getFullScaleGyroRange()) {
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

void GY88_Sensors::set_gyro_offsets(int gx, int gy, int gz){
    mpu.setXGyroOffset(gx);
    mpu.setYGyroOffset(gy);
    mpu.setZGyroOffset(gz);
}

void GY88_Sensors::set_accel_offsets(int ax, int ay, int az){
    mpu.setXAccelOffset(ax);
    mpu.setYAccelOffset(ay);
    mpu.setZAccelOffset(az);
}

void GY88_Sensors::get_offsets(){
    int ax = mpu.getXAccelOffset();
    int ay = mpu.getYAccelOffset();
    int az = mpu.getZAccelOffset();
    int gx = mpu.getXGyroOffset();
    int gy = mpu.getYGyroOffset();
    int gz = mpu.getZGyroOffset();

    Serial.println("Accel offsets: " + String(ax) + " , " + String(ay) + " , " + String(az));
    Serial.println("Gyro offsets: "  + String(gx) + " , " + String(gy) + " , " + String(gz));
}

void GY88_Sensors::calibrate(){

    get_offsets();
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);

    int16_t *offsets = mpu.GetActiveOffsets();
    set_accel_offsets(offsets[0], offsets[1], offsets[2]);
    set_gyro_offsets(offsets[3], offsets[4], offsets[5]);

    get_offsets();

    Serial.println("MPU calibrated!.");
}

void GY88_Sensors::print_xml(){
    float m[3][6] = {
                        {accel.x,       accel.y,        accel.z,    gyro.x,                 gyro.y,             gyro.z  },
                        {magnet.x,      magnet.y,       magnet.z,   magnet.heading,         0,                  0       },
                        {barom.temp,    barom.press,    barom.alt,  barom.sea_lvl_press,    0,                  0       }
                    };

    String str = "<Matrix type=\"np.float64\" origin=\"IMU\">";
    for(int i=0; i<3; i++){
        str+= "<ROW num=\""+String(i)+"\">";
        
        for(int j=0;j<6;j++){
            str+="<INDEX num=\""+String(j+1)+"\">"+String(m[i][j],4)+"</INDEX>";
        }

        str+="</ROW>";
    }
    str+="</Matrix>";

    Serial.println(str);
}


