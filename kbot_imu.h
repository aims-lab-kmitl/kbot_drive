#ifndef KBOT_IMU_H__
#define KBOT_IMU_H__

#include "MPU9250.h"

MPU9250 mpu;

void initIMU() {
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      delay(5000);
    }
  }
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  delay(5000);
  mpu.calibrateMag();
  
  mpu.verbose(false);
}

#endif
