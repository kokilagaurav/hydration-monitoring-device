#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "mbed.h"
#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

#define GSR_PIN A0
#define NUM_SAMPLES 50

// BLE UUIDs
BLEService sensorService("12345678-1234-5678-1234-56789abcdef0");
BLEFloatCharacteristic gsrCharacteristic("12345678-1234-5678-1234-56789abcdef1", BLERead | BLENotify);
BLEFloatCharacteristic accelCharacteristic("12345678-1234-5678-1234-56789abcdef2", BLERead | BLENotify);
BLEFloatCharacteristic gyroCharacteristic("12345678-1234-5678-1234-56789abcdef3", BLERead | BLENotify);
BLEFloatCharacteristic magCharacteristic("12345678-1234-5678-1234-56789abcdef4", BLERead | BLENotify);
BLEFloatCharacteristic irValueCharacteristic("12345678-1234-5678-1234-56789abcdef5", BLERead | BLENotify);  // IR only

int baseline = 0;
float gsr_alpha = 0.1;
float smooth_gsr = 0;

float gx_bias = 0, gy_bias = 0, gz_bias = 0;
float mx_offset = 0.1, my_offset = -0.2, mz_offset = 0.3;
float imu_alpha = 0.86;

unsigned long lastSensorRead = 0;
unsigned long lastMotionDetected = 0;
const unsigned long SENSOR_INTERVAL = 200;
const unsigned long INACTIVITY_TIMEOUT = 5 * 60 * 1000;
const float MOTION_THRESHOLD = 0.15;

bool deviceConnected = false;

void calibrateGSR() {
  Serial.println("Calibrating GSR...");
  int readings[NUM_SAMPLES];

  for (int i = 0; i < NUM_SAMPLES; i++) {
    readings[i] = analogRead(GSR_PIN);
    delay(5);
  }

  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = 0; j < NUM_SAMPLES - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        int temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  baseline = readings[NUM_SAMPLES / 2];
  Serial.println("GSR Calibration Done. Baseline: " + String(baseline));
}

void calibrateGyro() {
  Serial.println("Calibrating Gyroscope... Keep the device still.");
  int samples = 1000;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(3);
  }

  gx_bias = sumX / samples;
  gy_bias = sumY / samples;
  gz_bias = sumZ / samples;

  Serial.println("Gyro Calibration Done!");
}

void readAndSendSensorData() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  static float gx_f = 0, gy_f = 0, gz_f = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    gx -= gx_bias; gy -= gy_bias; gz -= gz_bias;
    gx_f = imu_alpha * gx_f + (1 - imu_alpha) * gx;
    gy_f = imu_alpha * gy_f + (1 - imu_alpha) * gy;
    gz_f = imu_alpha * gz_f + (1 - imu_alpha) * gz;
    if (abs(gx_f) < 0.02) gx_f = 0;
    if (abs(gy_f) < 0.02) gy_f = 0;
    if (abs(gz_f) < 0.02) gz_f = 0;
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    mx -= mx_offset;
    my -= my_offset;
    mz -= mz_offset;
  }

  int raw_gsr = analogRead(GSR_PIN);
  int calibrated_gsr = raw_gsr - baseline;
  smooth_gsr = gsr_alpha * calibrated_gsr + (1 - gsr_alpha) * smooth_gsr;
  int threshold = 5 + (baseline * 0.01);
  if (abs(smooth_gsr) < threshold) {
    smooth_gsr = 0;
  }

  float accel_mag = sqrt(ax * ax + ay * ay + az * az);
  float gyro_mag = sqrt(gx_f * gx_f + gy_f * gy_f + gz_f * gz_f);
  float mag_mag = sqrt(mx * mx + my * my + mz * mz);

  // BLE Send
  gsrCharacteristic.writeValue(smooth_gsr);
  accelCharacteristic.writeValue(accel_mag);
  gyroCharacteristic.writeValue(gyro_mag);
  magCharacteristic.writeValue(mag_mag);

  long irValue = particleSensor.getIR();
  irValueCharacteristic.writeValue((float)irValue);

  Serial.println("==== Sensor Readings ====");
  Serial.print("GSR (smoothed): "); Serial.println(smooth_gsr);
  Serial.print("Accel Magnitude: "); Serial.println(accel_mag);
  Serial.print("Gyro Magnitude: "); Serial.println(gyro_mag);
  Serial.print("Magnetometer Magnitude: "); Serial.println(mag_mag);
  Serial.print("IR Value: "); Serial.println(irValue);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found. Check wiring and power!");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }

  BLE.setLocalName("HyFlow");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(gsrCharacteristic);
  sensorService.addCharacteristic(accelCharacteristic);
  sensorService.addCharacteristic(gyroCharacteristic);
  sensorService.addCharacteristic(magCharacteristic);
  sensorService.addCharacteristic(irValueCharacteristic);  // IR only
  BLE.addService(sensorService);
  BLE.advertise();

  calibrateGyro();
  calibrateGSR();

  Serial.println("BLE Sensor ready. Waiting for Web Bluetooth connection...");
  lastSensorRead = millis();
  lastMotionDetected = millis();
}

void loop() {
  BLEDevice central = BLE.central();

  if (central && !deviceConnected) {
    deviceConnected = true;
    Serial.println("Device connected: " + central.address());
  }

  if (!central && deviceConnected) {
    deviceConnected = false;
    Serial.println("Device disconnected.");
  }

  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float accel_mag = sqrt(ax * ax + ay * ay + az * az);
    if (abs(accel_mag - 1.0) > MOTION_THRESHOLD) {
      lastMotionDetected = millis();
    }
  }

  if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
    readAndSendSensorData();
    lastSensorRead = millis();
  }

  if (millis() - lastMotionDetected >= INACTIVITY_TIMEOUT) {
    Serial.println("No motion for 5 minutes. Entering light sleep...");
    sleep();
    Serial.println("Woke from sleep.");
    lastMotionDetected = millis();
  } else {
    delay(500);
  }
}
