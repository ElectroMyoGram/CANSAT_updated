#include <math.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Simulating Structured IMU Data...");
}

void loop() {
  static float t = 0;  // Time variable

  // Simulate sinusoidal accelerometer data (m/s^2)
  float accelX = 9.8 * sin(t);  // Acceleration in X
  float accelY = 0.0;           // No acceleration in Y
  float accelZ = 9.8 * cos(t);  // Acceleration in Z (gravity effect)

  // Simulate constant gyroscope rotation (rad/s)
  float gyroX = 0.0;
  float gyroY = 0.5;  // Constant rotation about Y-axis
  float gyroZ = 0.0;

  // Simulate steady temperature (°C)
  float temp = 25.0;

  // Print simulated data
Serial.print("AccelX: ");
Serial.print(accelX);
Serial.print(" AccelY: ");
Serial.print(accelY);
Serial.print(" AccelZ: ");
Serial.println(accelZ);

  // Serial.print("Gyro (rad/s): X=");
  // Serial.print(gyroX);
  // Serial.print(",");
  // Serial.print(gyroY);
  // Serial.print(",");
  // Serial.println(gZ);

  // Serial.print("Temperature (°C): ");
  // Serial.println(temp);

  t += 0.1;  // Increment time (simulate ~10Hz data rate)
  delay(100);  // 100ms delay
}
