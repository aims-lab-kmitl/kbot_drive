#ifndef _MOTOR_CONTROL_H__
#define _MOTOR_CONTROL_H__

#include <Arduino.h>
#include "motor_pid.h"
#include "ESP32Encoder.h"

const int pulse_per_round = 360;
const int gear_ratio = 75;
const int encoder_mode = 4;

typedef struct {
  ESP32Encoder encoder;
  uint8_t encoder_a;
  uint8_t encoder_b;
  uint8_t pwm_channel;
  uint8_t pwm_pin;
  uint8_t dir_pin;
  int16_t speed_command;
  volatile int32_t sp_pose;
  volatile int32_t pv_pose;
  volatile float sp_speed;
  volatile float pv_speed;
} Motor_Data_Typedef;


void initMotorDriver(Motor_Data_Typedef *_motor_data, uint16_t _frequency);
void initEncoder(Motor_Data_Typedef *_motor_data);
void calculateMotorData(Motor_Data_Typedef *_motor_data, float _dt);
void motorSetSpeed(Motor_Data_Typedef *_motor_data, int16_t _speed);
void calculateMotorControl(Motor_Data_Typedef *_motor_data, PID_Data_Typedef *_pid_data, float _dt);
void runMotorControl(Motor_Data_Typedef *_motor_data, PID_Data_Typedef *_pid_data, float _dt);

#endif
