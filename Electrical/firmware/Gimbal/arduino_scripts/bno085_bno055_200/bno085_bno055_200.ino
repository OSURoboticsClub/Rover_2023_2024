#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ---------------------------
// BNO085 I2C Bus
// SDA = 17
// SCL = 16
// ---------------------------
TwoWire I2C_BNO085 = TwoWire(0);

// ---------------------------
// BNO055 I2C Bus
// SDA = 21
// SCL = 22
// ---------------------------
TwoWire I2C_BNO055 = TwoWire(1);

#define BNO085_ADDR 0x4A
#define BNO055_ADDR 0x28

#define SAMPLE_PERIOD_US 5000  // 200 Hz

BNO080 bno085;
Adafruit_BNO055 bno055 = Adafruit_BNO055(55, BNO055_ADDR, &I2C_BNO055);

unsigned long lastSampleUs = 0;

// Cached BNO085 accel values
float bno085_ax = 0;
float bno085_ay = 0;
float bno085_az = 0;

void setup() {
  Serial.begin(2000000);
  delay(1000);

  // ---------------------------
  // Initialize BNO085 bus
  // ---------------------------
  I2C_BNO085.begin(17, 16, 400000);

  // ---------------------------
  // Initialize BNO055 bus
  // ---------------------------
  I2C_BNO055.begin(21, 22, 400000);

  Serial.println("Starting IMUs...");

  // ---------------------------
  // Initialize BNO085
  // ---------------------------
  if (!bno085.begin(BNO085_ADDR, I2C_BNO085)) {
    Serial.println("BNO085 not detected");
    while (1);
  }

  // Request accel updates every 5ms = 200Hz
  bno085.enableAccelerometer(5);

  // ---------------------------
  // Initialize BNO055
  // ---------------------------
  if (!bno055.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  }

  delay(100);

  Serial.println("time_us,bno085_ax,bno085_ay,bno085_az,bno055_ax,bno055_ay,bno055_az");
}

void loop() {

  // ---------------------------
  // Read BNO085 when new data arrives
  // ---------------------------
  if (bno085.dataAvailable()) {
    bno085_ax = bno085.getAccelX();
    bno085_ay = bno085.getAccelY();
    bno085_az = bno085.getAccelZ();
  }

  unsigned long now = micros();

  // ---------------------------
  // Fixed 200Hz output loop
  // ---------------------------
  if ((unsigned long)(now - lastSampleUs) >= SAMPLE_PERIOD_US) {

    lastSampleUs += SAMPLE_PERIOD_US;

    imu::Vector<3> accel055 =
      bno055.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print(now);
    Serial.print(",");

    // BNO085
    Serial.print(bno085_ax, 6);
    Serial.print(",");
    Serial.print(bno085_ay, 6);
    Serial.print(",");
    Serial.print(bno085_az, 6);
    Serial.print(",");

    // BNO055
    Serial.print(accel055.x(), 6);
    Serial.print(",");
    Serial.print(accel055.y(), 6);
    Serial.print(",");
    Serial.println(accel055.z(), 6);
  }
}
