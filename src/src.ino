#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
 
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;
 
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
 
    Serial.println("Initializing Sensors...");
    if (!lsm6ds.begin_I2C()) {
        Serial.println("Failed to find LSM6DS IMU!");
        while (1) delay(10);
    }
    if (!lis3mdl.begin_I2C()) {
        Serial.println("Failed to find LIS3MDL Magnetometer!");
        while (1) delay(10);
    }
    Serial.println("Sensors initialized!");
}
 
void loop() {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);
 
    // Send raw sensor data (comma-separated)
    Serial.print(accel.acceleration.x); Serial.print(",");
    Serial.print(accel.acceleration.y); Serial.print(",");
    Serial.print(accel.acceleration.z); Serial.print(",");
    Serial.print(gyro.gyro.x); Serial.print(",");
    Serial.print(gyro.gyro.y); Serial.print(",");
    Serial.print(gyro.gyro.z); Serial.print(",");
    Serial.print(mag.magnetic.x); Serial.print(",");
    Serial.print(mag.magnetic.y); Serial.print(",");
    Serial.println(mag.magnetic.z);
 
    delay(10);  // Transmit at ~100Hz
}