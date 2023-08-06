#ifndef KBOT_IMU_H__
#define KBOT_IMU_H__

#include "kbot_imu.h"
#include "Wire.h"
#include "MPU6050_light.h"

MPU6050 mpu(Wire);

void initIMU(){
    Wire.begin();
    byte status = mpu.begin();
    while (status != 0) { }
    delay(1000);
    mpu.calcOffsets();
}

#endif