#include "motor_control.h"

void initMotorDriver(Motor_Data_Typedef *_motor_data,uint16_t _frequency){
  pinMode(_motor_data->pwm_pin,OUTPUT);
  pinMode(_motor_data->dir_pin,OUTPUT);
  ledcSetup(_motor_data->pwm_channel,_frequency,8);
  ledcAttachPin(_motor_data->pwm_pin,_motor_data->pwm_channel);
  ledcWrite(_motor_data->pwm_channel,0);
}

void initEncoder(Motor_Data_Typedef *_motor_data){
  pinMode(_motor_data->encoder_a,INPUT_PULLUP);
  pinMode(_motor_data->encoder_b,INPUT_PULLUP);
  ESP32Encoder::useInternalWeakPullResistors = UP;
  _motor_data->encoder.attachFullQuad(_motor_data->encoder_a,_motor_data->encoder_b);
  _motor_data->encoder.clearCount();
  _motor_data->encoder.setCount(0);
}

void calculateMotorData(Motor_Data_Typedef *_motor_data, float _dt){ 
  int32_t _pose = _motor_data->encoder.getCount();
  //Serial.println(_pose);
  _motor_data->encoder.setCount(0);
  _motor_data->encoder.clearCount();
  
  //get motor PV position
  _motor_data->pv_pose = (int32_t)_pose;

  float _rev = (float)_pose/((float)pulse_per_round * (float)encoder_mode  * (float)gear_ratio);
  //get speed in rpm
  _motor_data->pv_speed =  (_rev * 60.0) / _dt;
  //Serial.println(_motor_data->pv_speed);
}

void calculateMotorControl(Motor_Data_Typedef *_motor_data, PID_Data_Typedef *_pid_data, float _dt){
  float speed_command = 0.0;
  speed_command = calculatePID(_motor_data->sp_speed, _motor_data->pv_speed, _pid_data, _dt);
  
  if(speed_command > 255.0) speed_command = 255.0;
  if(speed_command < -255.0) speed_command = -255.0;
  
  if((_motor_data->sp_speed > 0.0) && (speed_command > 0.0)){
    motorSetSpeed(_motor_data, (int16_t)speed_command);
  }
  else if((_motor_data->sp_speed < 0.0) && (speed_command < 0.0)){
    motorSetSpeed(_motor_data, (int16_t)speed_command);
  }
  else if((int16_t)speed_command == 0){
    motorSetSpeed(_motor_data, 0);
    resetPIDData(_pid_data);
  }
  else{
    motorSetSpeed(_motor_data, 5);
    resetPIDData(_pid_data);
  }
}

void motorSetSpeed(Motor_Data_Typedef *_motor_data, int16_t _speed){
  int16_t speed;
  if(_speed > 0){
    speed = _speed;
    digitalWrite(_motor_data->dir_pin,LOW);  
  }
  else if(_speed < 0){
    speed = -1 * _speed;
    digitalWrite(_motor_data->dir_pin,HIGH);
  }
  else{
    speed = 0;
  }
  _motor_data->speed_command = speed;
  ledcWrite(_motor_data->pwm_channel,speed);
}

void runMotorControl(Motor_Data_Typedef *_motor_data, PID_Data_Typedef *_pid_data, float _dt){
  calculateMotorData(_motor_data, _dt);
  calculateMotorControl(_motor_data, _pid_data, _dt);
}
