#include <MPU9250_asukiaaa.h>
#include <BMP280.h>

#define I2C_ADDRESS_MPU 0x68
#define I2C_ADDRESS_BMP 0x76

BMP280 bmp(I2C_ADDRESS_BMP);
MPU9250_asukiaaa mpu(I2C_ADDRESS_MPU);

float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;



void setup() {
  Serial.begin(115200);

  Wire.begin(); // wire works with pin 18 (SDA) and 19 (SCL)

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  
  // mpu.setI2CBypassEnabled(true);

  bmp.begin();


  // You can set your own offset for mag values
  // mpu.magXOffset = -50;
  // mpu.magYOffset = -55;
  // mpu.magZOffset = -10;
}

void loop() {

  if (mpu.accelUpdate() == 0) {
    aX = mpu.accelX();
    aY = mpu.accelY();
    aZ = mpu.accelZ();
    aSqrt = mpu.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("\taccelY: " + String(aY));
    Serial.print("\taccelZ: " + String(aZ));
    Serial.print("\taccelSqrt: " + String(aSqrt));
  }

  if (mpu.gyroUpdate() == 0) {
    gX = mpu.gyroX();
    gY = mpu.gyroY();
    gZ = mpu.gyroZ();
    Serial.print("\tgyroX: " + String(gX));
    Serial.print("\tgyroY: " + String(gY));
    Serial.print("\tgyroZ: " + String(gZ));
  }

  if (mpu.magUpdate() == 0) {
    mX = mpu.magX();
    mY = mpu.magY();
    mZ = mpu.magZ();
    mDirection = mpu.magHorizDirection();
    Serial.print("\tmagX: " + String(mX));
    Serial.print("\tmagY: " + String(mY));
    Serial.print("\tmagZ: " + String(mZ));
    Serial.print("\thorizontalDirection: " + String(mDirection));
  }

  Serial.print("\tTemperature(*C): ");
  Serial.print(bmp.getTemperature());

  Serial.print("\tPressure(Pa): ");
  Serial.print(bmp.getPressure());


  // Serial.print("\tApproxAltitude(m): ");
  // Serial.print(bmp.getAltitude(1013.25)); // this should be adjusted to your local forcase

  Serial.println(""); // Add an empty line
  }
