#ifndef _INIT_MOTOR_H__
#define _INIT_MOTOR_H__

#include "define_io.h"
#include "motor_control.h"
#include "ESP32Encoder.h"
#include "kbot_serial_pid.h"

KBOTSERIAL kbot_serial_pid;

static ESP32Encoder encoder_left;
static ESP32Encoder encoder_right;
Motor_Data_Typedef motor_data_left;
Motor_Data_Typedef motor_data_right;

PID_Data_Typedef pid_data_left;
PID_Data_Typedef pid_data_right;

void initPinOutMotor(){
  motor_data_left.encoder = encoder_left;
  motor_data_left.encoder_a = ENCODER_LA_PIN;
  motor_data_left.encoder_b = ENCODER_LB_PIN;
  motor_data_left.pwm_channel = PWM_CHANNEL_LEFT;
  motor_data_left.pwm_pin = PWM_LEFT_PIN;
  motor_data_left.dir_pin = DIR_LEFT_PIN;
  
  motor_data_right.encoder = encoder_right;
  motor_data_right.encoder_a = ENCODER_RA_PIN;
  motor_data_right.encoder_b = ENCODER_RB_PIN;
  motor_data_right.pwm_channel = PWM_CHANNEL_RIGHT;
  motor_data_right.pwm_pin = PWM_RIGHT_PIN;
  motor_data_right.dir_pin = DIR_RIGHT_PIN;
}

void initMotorDevice(){
  initMotorDriver(&motor_data_left,FREQUENCY);
  initMotorDriver(&motor_data_right,FREQUENCY);

  initEncoder(&motor_data_left);
  initEncoder(&motor_data_right);

  resetPIDData(&pid_data_left);
  resetPIDData(&pid_data_right);
}

void initMotorParameter(){
  kbot_serial_pid.initPIDParameter(&pid_data_left,0);
  kbot_serial_pid.initPIDParameter(&pid_data_right,6);
}

void updatePIDData(){
  pid_data_left.kp  = kbot_serial_pid.readFlashData(0);
  pid_data_left.ki  = kbot_serial_pid.readFlashData(2);
  pid_data_left.kd  = kbot_serial_pid.readFlashData(4);
  pid_data_right.kp = kbot_serial_pid.readFlashData(6);
  pid_data_right.ki = kbot_serial_pid.readFlashData(8);
  pid_data_right.kd = kbot_serial_pid.readFlashData(10);
}

#endif
