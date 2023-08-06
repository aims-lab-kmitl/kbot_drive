#ifndef KBOT_ROS_H__
#define KBOT_ROS_H__

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <kbot_msgs/pid.h>
#include <kbot_msgs/lowlevel.h>

#include "kbot_motor.h"

#define LEFT        0
#define RIGHT       1
#define KP          0
#define KI          1
#define KD          2
#define MPS2RPM(x)  (x * 280.8616642798) // N = 60/(2*PI*r) * V
#define RPM2MPS(x)  (x * 0.003560471674) // V = (2*PI*r)/60 * N

ros::NodeHandle nh_;
ros::Time last_time_cmd_, time_now;
kbot_msgs::lowlevel kbot_lv_;

double goal_linear_velocity_;
double goal_angular_velocity_;
double wheel_speed_cmd_[2];
double set_speed_motor_[2];

void initPublishData(){
  kbot_lv_.header.frame_id = "base_link";
  kbot_lv_.speed_sp_length = 2;
  kbot_lv_.speed_fb_length = 2;
  kbot_lv_.pid_motor_left_length = 3;
  kbot_lv_.pid_motor_right_length = 3;

  kbot_lv_.speed_sp = (float *)malloc((sizeof(float))*kbot_lv_.speed_sp_length * 2);
  kbot_lv_.speed_fb = (float *)malloc((sizeof(float))*kbot_lv_.speed_fb_length * 2);
  kbot_lv_.pid_motor_left = (float *)malloc((sizeof(float))*kbot_lv_.pid_motor_left_length * 2);
  kbot_lv_.pid_motor_right = (float *)malloc((sizeof(float))*kbot_lv_.pid_motor_right_length * 2);
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg){
    last_time_cmd_ = nh_.now();
    goal_linear_velocity_  = cmd_vel_msg.linear.x;
    goal_angular_velocity_ = cmd_vel_msg.angular.z;

    wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * 0.195 / 2);
    wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * 0.195 / 2);
    
    set_speed_motor_[LEFT]  = MPS2RPM(wheel_speed_cmd_[LEFT]);
    set_speed_motor_[RIGHT] = MPS2RPM(wheel_speed_cmd_[RIGHT]);
}

void PIDDataCallBack(const kbot_msgs::pid& pid_msg){
  char log_msg[50];
  if((String)pid_msg.motor == "L"){
    kbot_serial_pid.writeFlashData(0, pid_msg.kp);
    kbot_serial_pid.writeFlashData(2, pid_msg.ki);
    kbot_serial_pid.writeFlashData(4, pid_msg.kd);

    sprintf(log_msg, "Motor Left kp: %d ki: %d kd: %d", pid_msg.kp, pid_msg.ki, pid_msg.kd);
  }
  else if((String)pid_msg.motor == "R"){
    kbot_serial_pid.writeFlashData(6, pid_msg.kp);
    kbot_serial_pid.writeFlashData(8, pid_msg.ki);
    kbot_serial_pid.writeFlashData(10, pid_msg.kd);

    sprintf(log_msg, "Motor Right kp: %d ki: %d kd: %d", pid_msg.kp, pid_msg.ki, pid_msg.kd);
  }
  else
    sprintf(log_msg, "Error Received PID Data");
  nh_.loginfo(log_msg);
}

void updatePIDDataROS(){
  kbot_lv_.pid_motor_left[KP]  = kbot_serial_pid.readFlashData(0);
  kbot_lv_.pid_motor_left[KI]  = kbot_serial_pid.readFlashData(2);
  kbot_lv_.pid_motor_left[KD]  = kbot_serial_pid.readFlashData(4);
  kbot_lv_.pid_motor_right[KP] = kbot_serial_pid.readFlashData(6);
  kbot_lv_.pid_motor_right[KI] = kbot_serial_pid.readFlashData(8);
  kbot_lv_.pid_motor_right[KD] = kbot_serial_pid.readFlashData(10);
}

ros::Publisher kbot_lv_pub_("/kbot/micro/data", &kbot_lv_);
ros::Subscriber<kbot_msgs::pid> kbot_pid_sub_("/kbot/micro/pid_tuning", &PIDDataCallBack);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub_("/cmd_vel", &commandVelocityCallback);

#endif
