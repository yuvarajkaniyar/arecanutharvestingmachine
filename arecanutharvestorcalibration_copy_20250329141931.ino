#include "Wire.h"
#include "mpu9250.h"
#include <math.h>

Mpu9250 imu(&Wire, 0x68);

// Variables for sensor data
float ax, ay, az;
float gx, gy, gz;

// Timing variables
unsigned long lastTime, currentTime;
float dt;

// Complementary filter variables for roll and pitch (in degrees)
float angleX = 0.0, angleY = 0.0;    // Filtered angles
const float alpha = 0.98;            // Complementary filter coefficient (tune as needed)

void setup() {
  Serial.begin(115200);
  while (!Serial) {} // Wait for serial to be ready

  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication

  if (!imu.Begin()) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("MPU9250 Initialized!");

  // Initialize timing
  lastTime = micros();
}

void loop() {
  // Calculate elapsed time (in seconds)
  currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  
  // Read sensor data
  if (imu.Read()) {
    // Get accelerometer and gyroscope readings
    ax = imu.accel_x_mps2();
    ay = imu.accel_y_mps2();
    az = imu.accel_z_mps2();
    gx = imu.gyro_x_radps();
    gy = imu.gyro_y_radps();
    gz = imu.gyro_z_radps();

    // Compute roll and pitch from accelerometer (in degrees)
    // Using standard equations: roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay*ay + az*az))
    float accelRoll  = atan2(ay, az) * 180 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

    // Integrate gyroscope data (convert rad/s to deg/s: multiply by 180/PI)
    float gyroRollRate  = gx * 180 / PI;
    float gyroPitchRate = gy * 180 / PI;

    // Complementary filter: combine gyro integration with accelerometer measurement
    angleX = alpha * (angleX + gyroRollRate * dt) + (1.0 - alpha) * accelRoll;
    angleY = alpha * (angleY + gyroPitchRate * dt) + (1.0 - alpha) * accelPitch;

    // Output the computed roll and pitch angles (for debugging or PID input)
    Serial.print("Roll: ");
    Serial.print(angleX, 2);
    Serial.print("\tPitch: ");
    Serial.print(angleY, 2);
    Serial.print("\tLoop dt: ");
    Serial.print(dt, 6);
    Serial.println(" s");
  }
}
