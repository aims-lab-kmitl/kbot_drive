#include "motor_pid.h"

float calculatePID(float _sp, float _pv, PID_Data_Typedef *_pid_data, float _dt) {
  _pid_data->err = _sp - _pv;
  /*limit error*/
  if (_pid_data->err > 1000.0) _pid_data->err = 1000.0;
  if (_pid_data->err < -1000.0) _pid_data->err = -1000.0;

  _pid_data->sum_err += _pid_data->err;
  float ret = (_pid_data->kp * _pid_data->err) +
              (_pid_data->ki * _pid_data->sum_err * _dt) +
              (_pid_data->kd * ((_pid_data->err - _pid_data->err_1) / _dt));

  _pid_data->err_1 = _pid_data->err;

  /*limit output*/
  if (ret > 255.0) ret = 255.0;
  if (ret < -255.0) ret = -255.0;

  return ret;
}

void resetPIDData(PID_Data_Typedef *_pid_data) {
  _pid_data->err = 0;
  _pid_data->err_1 = 0;
  _pid_data->sum_err = 0;
}

void setupPIDParameter(PID_Data_Typedef *_pid_data, float _kp, float _ki, float _kd) {
  _pid_data->kp = _kp;
  _pid_data->ki = _ki;
  _pid_data->kd = _kd;
}
