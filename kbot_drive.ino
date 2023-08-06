#include "kbot_motor.h"
#include "kbot_ros.h"
#include "kbot_imu.h"

unsigned long t;
unsigned long tTime[2] = {0, 0};

void setup() {
  // imu
  initIMU();

  // motor
  kbot_serial_pid.begin();
  initPinOutMotor();
  initMotorDevice();
  initMotorParameter();

  // ros
  nh_.initNode();
  nh_.advertise(kbot_lv_pub_);
  nh_.subscribe(kbot_pid_sub_);
  nh_.subscribe(cmd_vel_sub_);
  initPublishData();
}

void loop() {
  time_now = nh_.now();
  t = millis();
  if(millis() - tTime[0] > 10 ){
    updatePIDData();
    updatePIDDataROS();
    motor_data_left.sp_speed  = set_speed_motor_[LEFT];
    motor_data_right.sp_speed = set_speed_motor_[RIGHT];
    
    if((time_now - last_time_cmd_).toSec() > 1.0){
      motor_data_left.sp_speed = 0.0;
      motor_data_right.sp_speed = 0.0;
    }

    runMotorControl(&motor_data_left, &pid_data_left, 0.01);
    runMotorControl(&motor_data_right, &pid_data_right, 0.01);

    kbot_lv_.speed_sp[LEFT]   = motor_data_left.sp_speed;
    kbot_lv_.speed_sp[RIGHT]  = motor_data_right.sp_speed;
    kbot_lv_.speed_fb[LEFT]   = motor_data_left.pv_speed;
    kbot_lv_.speed_fb[RIGHT]  = motor_data_right.pv_speed;

    tTime[0] = t;
  }

  if(millis() - tTime[1] > 30){
    kbot_lv_.header.stamp = nh_.now();
    kbot_lv_.theta = mpu.getAngleZ();
    kbot_lv_pub_.publish(&kbot_lv_);

    tTime[1] = t;
  }
  nh_.spinOnce();
}
