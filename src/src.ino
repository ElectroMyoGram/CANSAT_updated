#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LIS3MDL.h>
#include <ArduinoEigenDense.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

using namespace Eigen;



// IMU (Accelerometer & Gyro)
Adafruit_LSM6DSOX lsm6ds;

// Magnetometer
Adafruit_LIS3MDL lis3mdl;

// GPS
static const int RXPIN = 44, TXPin = 3;
static const uint32_t GPSBAUD = 9600;

TinyGpsPlus gps;
SoftwareSerial ss(RXPIN, TXPIN) //rx, tx





// Function prototypes
void setupIMU();
void setupMagnetometer();
void getIMUData();
void getMagnetometerData();

const float alpha = 0.98;
unsigned long prev_time = 0;
float roll = 0.0, pitch = 0.0;


void setup() {
    Serial.begin(115200);

    while (!Serial) delay(10); // Wait for serial console
    Serial.begin(115200);
    Serial.println("Eigen is working!");
    Serial.println("Initializing Sensors...");

    // Setup sensors
    setupIMU();
    setupMagnetometer();
    setupGPS();


//     GPS_Serial.begin(GPS_BAUD, 1, 2);
//     GPS.begin(GPS_BAUD);
//     GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//     GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//     Serial.println("GPS Initialized...");
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0; // Convert to seconds
  prev_time = current_time;

  getGPSData
  delay(10);

}

/* ============================ */
/*        SENSOR FUNCTIONS      */
/* ============================ */

void setupGPS() {
  ss.begin(GPS_BAUD);

  Serial.println(F("SatelliteTracker.ino"));
  Serial.println(F("Monitoring satellite location and signal strength using TinyGPSCustom"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void getGPSData() {
  if (ss.available() > 0){
    gps.encode(ss.read())
  }
  if (gps.location.isUpdated()){
    Serial.print("Lat="); Serial.print(gps.location.lat(), 6);
    Serial.print("LNG="); Serial.print(gps.location.lng(), 6);
  }
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

void getIMUData() {
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

