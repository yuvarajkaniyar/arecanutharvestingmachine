# Automatic Arecanut Harvesting Machine

## Overview
The automatic areca nut harvesting machine addresses the challenges of manual harvesting by offering a safer, more efficient, and crop-friendly solution. Using drone-like propulsion technology combined with cutting-edge automation, the machine detaches areca nuts from trees without causing harm to companion crops like pepper. Initially, the prototype will be remote-controlled, with plans for full automation in future iterations.

## Problem Statement
Arecanut is one of the top-grown commercial crops in Karnataka, significantly impacting the economy of the Malnad and Karavali regions. The areca nut industry faces challenges in harvesting due to labor shortages, safety risks, and high manual labor costs. This machine aims to reduce dependency on manual labor, improve safety, and offer a cost-effective alternative for farmers.

## Impact
1. **Increased Productivity** - 7–10x improvement over manual methods.
2. **Reduction in Labor Dependency** - Decreases labor costs by 60-70%.
3. **Enhanced Worker Safety** - Eliminates the need for climbing tall trees.
4. **Cost Savings** - Reduces per-acre harvesting costs by up to 80%.
5. **Improved Quality of Harvest** - Ensures timely collection of areca nuts.
6. **Environmental Benefits** - Lower carbon footprint compared to petrol-based alternatives.

## Technical Features
- **Drone Propulsion System** - Uses four propellers for hovering and maneuvering.
- **Linear Motion System with Locking Mechanism** - Ensures precise cutting.
- **Cutting Mechanism** - Blade mounted at the top for clean detachment.
- **Microcontroller & Vision System** - Raspberry Pi 3 with a camera for detection.
- **Speed & Efficiency** - Operates at 0.6 m/s.
- **Future Development** - Attachment to collect areca nuts safely.

## Work Done in the 1st Month
- Understood **bare metal programming basics using STM32**.
- [certification Link](https://www.linkedin.com/posts/yuvaraj-kaniyar_stm32-embeddedsystems-baremetalprogramming-activity-7304115431663931392-RNuB?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD3Z9zcBoqSkrvq9UVeUqWN8_bo1XDrhz9Y)
- Sourced **MPU9250 9 axis accelerometer for Rs 590 for Testing with UVCE GA bill**.
    ![MPU9250](https://github.com/yuvarajkaniyar/arecanutharvestingmachine/blob/main/mpu9250.jpg)
- Sourced **STM32F103C8 Microcontroller and STLINK V2 bootloader**.
  ![stm32](https://github.com/yuvarajkaniyar/arecanutharvestingmachine/blob/main/stm32.jpg)
    
- Connected **STM32 with MPU 9250** to obtain pitch, yaw, and roll for calibration.
  ```c
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
 
![OUTPUT](https://github.com/yuvarajkaniyar/arecanutharvestingmachine/blob/main/serialmonitormpu9250.jpg)
- Purchased **propellers from offline sources**.
  ![PROPELLOR](https://github.com/yuvarajkaniyar/arecanutharvestingmachine/blob/main/PROFEELORS.jpg)
- Sourced **Raspberry Pi 4 for testing**.
  ![Raspberypi](https://github.com/yuvarajkaniyar/arecanutharvestingmachine/blob/main/raspberrypi4.jpg)
- Completed **3D modeling of the chassis**.
- Identified that the material available in Marvel is weak; **stronger material is needed for 3D printing**.
  
 ## References
[Youtube resource](https://youtube.com/playlist?list=PL0K4VDicBzshwCpUHzIB6hOLQVkDFHbxC&si=feCqDbxrZ5yR-K9O)
[Linkedin Inspiration](https://www.linkedin.com/posts/dipanshu-dhote-246aa9331_esp12e-intense-stability-activity-7309827075018813442-WwC2?utm_source=share&utm_medium=member_desktop&rcm=ACoAAD3Z9zcBoqSkrvq9UVeUqWN8_bo1XDrhz9Y)
[Chassis Model Link](https://a360.co/4h9sC4G)


## Implementation Details
**Tools & Resources:**
- Raspberry Pi 3 (processing)
- Drone motors & propellers
- Linear motion motor with locking mechanism
- Rechargeable battery power source

**Development Steps:**
1. Assemble drone frame.
2. Integrate linear motor & cutting blade.
3. Program Raspberry Pi for control & height detection.
4. Test remote control functionality.

## Challenges & Mitigation
- **Calibration** → Develop a stable version.
- **Weight Management** → Use lightweight materials.
- **Detection Accuracy** → Improve with high-quality cameras.
- **Material Strength for 3D Printing** → Source a stronger material to improve durability.

## Timelines
- **Prototype Assembly:** 1.5 months
- **Testing & Iteration:** 0.5 months
- **Final Prototype Delivery:** 3 months

## Cost & Pricing
- **Current Manufacturing Cost:** ₹15,000 per unit
- **Future Cost Reduction:** ₹10,000 per unit
- **Proposed Market Price:** ₹20,000 (affordable for small & medium farmers)

## Target Audience
- Arecanut Farmers
- Agricultural Contractors
- Government Agencies
- Agricultural Cooperatives

## Outcome
- Efficient & safe arecanut harvesting.
- Remote-controlled operation.
- Foundation for future automation.

## Limitations
- Initial reliance on remote control.
- Battery efficiency constraints in wireless mode.
- Handling dense areca nut clusters may require refinements.



