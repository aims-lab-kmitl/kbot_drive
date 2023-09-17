#ifndef KBOT_ROS_H__
#define KBOT_ROS_H__

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <kbot_msgs/motor.h>
#include <kbot_msgs/pid.h>
#include <kbot_msgs/rollpitchyaw.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>

#include "kbot_motor.h"

#define LEFT 0
#define RIGHT 1
#define KP 0
#define KI 1
#define KD 2
#define MPS2RPM(x) (x * 280.8616642798) // N = 60/(2*PI*r) * V
#define RPM2MPS(x) (x * 0.003560471674) // V = (2*PI*r)/60 * N

ros::NodeHandle nh_;
ros::Time last_time_cmd_, time_now;
kbot_msgs::motor motor_;
kbot_msgs::rollpitchyaw rpy_;
sensor_msgs::Imu imu_;
sensor_msgs::MagneticField mag_;

double goal_linear_velocity_;
double goal_angular_velocity_;
double wheel_speed_cmd_[2];
double set_speed_motor_[2];

void initPublishData()
{
  motor_.speed_sp_length = 2;
  motor_.speed_fb_length = 2;
  motor_.pid_motor_left_length = 3;
  motor_.pid_motor_right_length = 3;

  motor_.speed_sp = (float *)malloc((sizeof(float)) * motor_.speed_sp_length * 2);
  motor_.speed_fb = (float *)malloc((sizeof(float)) * motor_.speed_fb_length * 2);
  motor_.pid_motor_left = (float *)malloc((sizeof(float)) * motor_.pid_motor_left_length * 2);
  motor_.pid_motor_right = (float *)malloc((sizeof(float)) * motor_.pid_motor_right_length * 2);

  imu_.header.frame_id = "imu_link";
  imu_.orientation.w = 0.0;
  imu_.orientation.x = 0.0;
  imu_.orientation.y = 0.0;
  imu_.orientation.z = 0.0;
  imu_.orientation_covariance[0] = 0.0025;
  imu_.orientation_covariance[1] = 0;
  imu_.orientation_covariance[2] = 0;
  imu_.orientation_covariance[3] = 0;
  imu_.orientation_covariance[4] = 0.0025;
  imu_.orientation_covariance[5] = 0;
  imu_.orientation_covariance[6] = 0;
  imu_.orientation_covariance[7] = 0;
  imu_.orientation_covariance[8] = 0.0025;

  imu_.angular_velocity.x = 0.0;
  imu_.angular_velocity.y = 0.0;
  imu_.angular_velocity.z = 0.0;
  imu_.angular_velocity_covariance[0] = 0.02;
  imu_.angular_velocity_covariance[1] = 0;
  imu_.angular_velocity_covariance[2] = 0;
  imu_.angular_velocity_covariance[3] = 0;
  imu_.angular_velocity_covariance[4] = 0.02;
  imu_.angular_velocity_covariance[5] = 0;
  imu_.angular_velocity_covariance[6] = 0;
  imu_.angular_velocity_covariance[7] = 0;
  imu_.angular_velocity_covariance[8] = 0.02;

  imu_.linear_acceleration.x = 0.0;
  imu_.linear_acceleration.y = 0.0;
  imu_.linear_acceleration.z = 0.0;
  imu_.linear_acceleration_covariance[0] = 0.04;
  imu_.linear_acceleration_covariance[1] = 0;
  imu_.linear_acceleration_covariance[2] = 0;
  imu_.linear_acceleration_covariance[3] = 0;
  imu_.linear_acceleration_covariance[4] = 0.04;
  imu_.linear_acceleration_covariance[5] = 0;
  imu_.linear_acceleration_covariance[6] = 0;
  imu_.linear_acceleration_covariance[7] = 0;
  imu_.linear_acceleration_covariance[8] = 0.04;

  mag_.header.frame_id = "magnetic_link";
  mag_.magnetic_field.x = 0.0;
  mag_.magnetic_field.y = 0.0;
  mag_.magnetic_field.z = 0.0;
  mag_.magnetic_field_covariance[0] = 0.0048;
  mag_.magnetic_field_covariance[1] = 0;
  mag_.magnetic_field_covariance[2] = 0;
  mag_.magnetic_field_covariance[3] = 0;
  mag_.magnetic_field_covariance[4] = 0.0048;
  mag_.magnetic_field_covariance[5] = 0;
  mag_.magnetic_field_covariance[6] = 0;
  mag_.magnetic_field_covariance[7] = 0;
  mag_.magnetic_field_covariance[8] = 0.0048;

  rpy_.roll   = 0.0;
  rpy_.pitch  = 0.0;
  rpy_.yaw    = 0.0;
}

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
  last_time_cmd_ = nh_.now();
  goal_linear_velocity_ = cmd_vel_msg.linear.x;
  goal_angular_velocity_ = cmd_vel_msg.angular.z;

  wheel_speed_cmd_[LEFT] = goal_linear_velocity_ - (goal_angular_velocity_ * 0.195 / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * 0.195 / 2);

  set_speed_motor_[LEFT] = MPS2RPM(wheel_speed_cmd_[LEFT]);
  set_speed_motor_[RIGHT] = MPS2RPM(wheel_speed_cmd_[RIGHT]);
}

void PIDDataCallBack(const kbot_msgs::pid &pid_msg)
{
  char log_msg[50];
  if ((String)pid_msg.motor == "L")
  {
    kbot_serial_pid.writeFlashData(0, pid_msg.kp);
    kbot_serial_pid.writeFlashData(2, pid_msg.ki);
    kbot_serial_pid.writeFlashData(4, pid_msg.kd);

    sprintf(log_msg, "Motor Left kp: %d ki: %d kd: %d", pid_msg.kp, pid_msg.ki, pid_msg.kd);
  }
  else if ((String)pid_msg.motor == "R")
  {
    kbot_serial_pid.writeFlashData(6, pid_msg.kp);
    kbot_serial_pid.writeFlashData(8, pid_msg.ki);
    kbot_serial_pid.writeFlashData(10, pid_msg.kd);

    sprintf(log_msg, "Motor Right kp: %d ki: %d kd: %d", pid_msg.kp, pid_msg.ki, pid_msg.kd);
  }
  else
    sprintf(log_msg, "Error Received PID Data");
  nh_.loginfo(log_msg);
}

void updatePIDDataROS()
{
  motor_.pid_motor_left[KP] = kbot_serial_pid.readFlashData(0);
  motor_.pid_motor_left[KI] = kbot_serial_pid.readFlashData(2);
  motor_.pid_motor_left[KD] = kbot_serial_pid.readFlashData(4);
  motor_.pid_motor_right[KP] = kbot_serial_pid.readFlashData(6);
  motor_.pid_motor_right[KI] = kbot_serial_pid.readFlashData(8);
  motor_.pid_motor_right[KD] = kbot_serial_pid.readFlashData(10);
}

void commandCalibrationCallback(const std_msgs::Empty &calibration_msg);

ros::Publisher motor_pub_("/kbot/micro/motor", &motor_);
ros::Publisher imu_pub_("/kbot/micro/imu_raw", &imu_);
ros::Publisher mag_pub_("/kbot/micro/magnetic_raw", &mag_);
ros::Publisher rpy_pub_("/kbot/micro/rpy", &rpy_);
ros::Subscriber<kbot_msgs::pid> pib_sub_("/kbot/micro/pid_tuning", &PIDDataCallBack);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_("/cmd_vel", &commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> calibration_sub_("/kbot/calibration", &commandCalibrationCallback);

#endif
