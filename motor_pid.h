#ifndef _MOTOR_PID_H__
#define _MOTOR_PID_H__

typedef struct {
  float kp;
  float ki;
  float kd;
  float err;
  float err_1;
  float sum_err;
} PID_Data_Typedef;

void setupPIDParameter(PID_Data_Typedef *_pid_data, float _kp, float _ki, float _kd);
float calculatePID(float _sp, float _pv, PID_Data_Typedef *_pid_data, float _dt);
void resetPIDData(PID_Data_Typedef *_pid_data);

#endif
