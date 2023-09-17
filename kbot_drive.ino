#include "kbot_motor.h"
#include "kbot_ros.h"
#include "kbot_imu.h"

unsigned long t;
unsigned long tTime[3] = {0, 0, 0};

void setup()
{
  // imu
  initIMU();

  // motor
  kbot_serial_pid.begin();
  initPinOutMotor();
  initMotorDevice();
  initMotorParameter();

  // ros
  nh_.initNode();
  nh_.advertise(motor_pub_);
  nh_.advertise(imu_pub_);
  nh_.advertise(mag_pub_);
  nh_.advertise(rpy_pub_);
  nh_.subscribe(pib_sub_);
  nh_.subscribe(cmd_vel_sub_);
  nh_.subscribe(calibration_sub_);
  initPublishData();
}

void loop()
{
  time_now = nh_.now();
  t = millis();
  mpu.update();
  if (t - tTime[0] > 10)
  {
    updatePIDData();
    updatePIDDataROS();
    motor_data_left.sp_speed = set_speed_motor_[LEFT];
    motor_data_right.sp_speed = set_speed_motor_[RIGHT];

    if ((time_now - last_time_cmd_).toSec() > 1.0)
    {
      motor_data_left.sp_speed = 0.0;
      motor_data_right.sp_speed = 0.0;
    }

    runMotorControl(&motor_data_left, &pid_data_left, 0.01);
    runMotorControl(&motor_data_right, &pid_data_right, 0.01);

    motor_.speed_sp[LEFT] = motor_data_left.sp_speed;
    motor_.speed_sp[RIGHT] = motor_data_right.sp_speed;
    motor_.speed_fb[LEFT] = motor_data_left.pv_speed;
    motor_.speed_fb[RIGHT] = motor_data_right.pv_speed;

    tTime[0] = t;
  }

  if (t - tTime[1] > 30)
  {
    imu_.orientation.w = mpu.getQuaternionW();
    imu_.orientation.x = mpu.getQuaternionX();
    imu_.orientation.y = mpu.getQuaternionY();
    imu_.orientation.z = mpu.getQuaternionZ();
    imu_.angular_velocity.x = mpu.getGyroX();
    imu_.angular_velocity.y = mpu.getGyroY();
    imu_.angular_velocity.z = mpu.getGyroZ();
    imu_.linear_acceleration.x = mpu.getAccX();
    imu_.linear_acceleration.y = mpu.getAccY();
    imu_.linear_acceleration.z = mpu.getAccZ();

    mag_.magnetic_field.x = mpu.getMagX();
    mag_.magnetic_field.y = mpu.getMagY();
    mag_.magnetic_field.z = mpu.getMagZ();

    rpy_.roll = mpu.getRoll();
    rpy_.pitch = mpu.getPitch();
    rpy_.yaw = mpu.getYaw();

    tTime[1] = t;
  }

  if (t - tTime[2] > 50)
  {
    motor_pub_.publish(&motor_);

    imu_.header.stamp = time_now;
    imu_pub_.publish(&imu_);

    mag_.header.stamp = time_now;
    mag_pub_.publish(&mag_);

    rpy_pub_.publish(&rpy_);

    tTime[2] = t;
  }
  nh_.spinOnce();
}

void commandCalibrationCallback(const std_msgs::Empty &calibration_msg)
{
  (void)(calibration_msg);

  nh_.loginfo("Accel Gyro calibration.");

  mpu.verbose(true);
  mpu.calibrateAccelGyro();

  nh_.loginfo("Mag calibration.");

  mpu.calibrateMag();
  mpu.verbose(false);

  nh_.loginfo("Calibration complete.");
}
