#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LIS3MDL.h>
#include <ArduinoEigenDense.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "driver/uart.h"

using namespace Eigen;

// IMU (Accelerometer & Gyro)
Adafruit_LSM6DSOX lsm6ds;
// Magnetometer
Adafruit_LIS3MDL lis3mdl;

// GPS
static const int RXPIN = 44, TXPIN = 43;
static const uint32_t GPS_BAUD = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); //rx, tx


// Function prototypes
void setupIMU();
void setupMagnetometer();
void setupGPS();

void getIMUData();
void getMagnetometerData();

//time syncing
unsigned long prev_time = 0;
const float dt = 0.01;

//EKF variables
VectorXd X(7);
MatrixXd P(4,4), Q(4,4), R(6,6);

float cAtt0 = 0.001;
float cbias0 = 0.0001;

//process noise 
const float MatrixXdnProcAtt = 0.00005;
const float nProcBias = 0.000001;
const int stateSize = 7;
MatrxXd Q = MatrixXd::Zero(stateSize, stateSize);

//measurement noise Matrix
const float nMeasAcc = 0.05;
const float nMeasMag = 0.02;
const int measSize = 6;
MatrixXd R = MatrixXd::Zero(measSize, measSize);

//consts
const float g = 9.81;
//low pass filter measuerments
float p = 0.0, q = 0.0, r = 0.0;
ax = 0.0, ay = 0.0, az = 0.0;
mx = 0.0, my = 0.0, mz = 0.0;

//low pass filter coeficents
const GyrCoef = 0.7;
const AccCoef = 0.9;
const MagCoef = 0.4;



void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);


  Serial.println("Eigen is working!");
  Serial.println("Initializing Sensors...");

  //Setup sensors
  setupIMU();
  setupMagnetometer();
  // setupGPS();

  Serial.println("done magnetoemter");

  //identity quaternian
  X(0) = 1;

  //process noise covariance
  for (int i = 0; i < 4; i++){
    Q(i, i) = nProcAtt * nProcAtt;
  }
  for (int i = 4; i < stateSize; i++){
    Q(i, i) = nProcBias * nProcBias
  }

  //measurement covariance
  for (int i = 0; i < 3; i++){
    R(i, i) = nMeasAcc * nMeasAcc;
  }
  for (int i = 3; i < measSize; i++){
    R(i, i) = nMeasMag * nMeasMag;
  }
}

void loop() {
  unsigned long current_time = millis();
  dt = (current_time - prev_time) / 1000;
  prev_time = current_time;
  


  sensors_event_t accel, gyro, mag, temp;
  lsm6ds.getEvent(&accel, &gyro, temp);
  lis3mdl.getEvent(&mag);
  
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;

  float mx = mag.magnetic.x;
  float my= mag.magnetic.y;
  float mz = mag.magnetic.z;

  p = GyrCoef * p + (1 - GyroCoef) * gx;
  q = GyroCoef * q + (1 - GyroCoef) * gy;
  r = GyroCoef * r + (1 - GyroCoef) * gz;

  ax = AccCoef * ax + (1 - AccCoef) * ax;
  ay = AccCoef * ay + (1 - AccCoef) * ay;
  az = AccCoef * az + (1 - AccCoef) * az;

  mx = MagCoef * mx + (1 - MagCoef) * mx;
  my = MagCoef * my + (1 - MagCoef) * my;
  mz = MagCoef * mz + (1 - MagCoef) * mz;

  //normalize magneteoitoemtioer readings
  




  //prediction
  MatrixXd f = MatrixXd::Identity(3,3)
  X = F*X;

  //predict covariance
  P = F * P * F.transpose() + Q;

  //update
  VectorXd Z(3);
  Z << ax, ay, az, mx, my, mz;
  MatrixXd H = MatrixXd::Zero(6,9);
  H.block(0,3,3,3) = MatrixXd::Identity(3,3);
  MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  X = X + K * (Z - H * X);
  P = (MatrixXd::Identity(9,9) - K * H) * P;
  
  
  Serial.print(X(0)); LoRa.print(",");
  Serial.print(X(1)); LoRa.print(",");
  Serial.print(X(2)); LoRa.print(",");
  Serial.print(X(6)); LoRa.print(",");
  Serial.print(X(7)); LoRa.print(",");
  
  delay(10);

}

/* ============================ */
/*        SENSOR FUNCTIONS      */
/* ============================ */

void setupGPS() {
    Serial.println("Reading GPS data...");

    gpsSerial.begin(9600, SERIAL_8N1, RXPIN, TXPIN);
    uart_set_pin(UART_NUM_2, TXPIN, RXPIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void getGPSData() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    Serial.print("Fix Status: ");
    Serial.println(gps.satellites.value() > 0 ? "FIXED ✅" : "NO FIX ❌");

    Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    Serial.print("Speed (km/h): "); Serial.println(gps.speed.kmph());
    Serial.println("----------------------");

    delay(1000);
}


// Setup function for IMU (Accelerometer & Gyroscope)
void setupIMU() {
    if (!lsm6ds.begin_I2C()) {
        Serial.println("Failed to find LSM6DS IMU!");
        while (1) delay(10);
    }
    Serial.println("LSM6DS IMU Found!");
}

// Setup function for Magnetometer
void setupMagnetometer() {
    if (!lis3mdl.begin_I2C()) {
        Serial.println("Failed to find LIS3MDL Magnetometer!");
        while (1) delay(10);
    }

    Serial.println("LIS3MDL Magnetometer Found!");
    
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setIntThreshold(500);
    lis3mdl.configInterrupt(false, false, true, true, false, true);
}

void getIMUData(float dt) {
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  sensors_event_t mag;
  lis3mdl.getEvent(&mag);

  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  float gx = gyro.gyro.x * (180.0 / M_PI);
  float gy = gyro.gyro.y * (180.0 / M_PI);
  float gz = gyro.gyro.z * (180.0 / M_PI);

  float roll_acc = atan2(ay, az) * (180.0 / M_PI);
  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI);

  roll = alpha * (roll + gx * dt) + (1 - alpha) * roll_acc;
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc;

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);

  // Serial.print(",");
  // Serial.print(yaw);
  Serial.println();
}

// Function to get Magnetometer Data
void getMagnetometerData() {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);

    Serial.print("Mag X: "); Serial.print(mag.magnetic.x, 4);
    Serial.print(" Y: "); Serial.print(mag.magnetic.y, 4);
    Serial.print(" Z: "); Serial.println(mag.magnetic.z, 4);
}

