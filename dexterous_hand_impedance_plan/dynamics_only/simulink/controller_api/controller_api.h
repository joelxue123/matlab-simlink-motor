#ifndef CONTROLLER_API_H_
#define CONTROLLER_API_H_

#include "../Controller_grt_rtw/usr_pid.h"

#define CONTROLLER_Q_ANGLE_FRAC_BITS   (12)
#define CONTROLLER_Q_SPEED_FRAC_BITS   (8)
#define CONTROLLER_Q_ACCEL_FRAC_BITS   (4)
#define CONTROLLER_Q_TORQUE_FRAC_BITS  (13)

typedef struct {
  int16_T q_ref;
  int16_T q;
  int16_T qdot;
  int16_T qddot;
  int16_T tau_prev;
  uint8_T mode;
} ControllerApiInput;

typedef struct {
  int16_T tau_cmd;
  int16_T tau_load_hat;
} ControllerApiOutput;

void ControllerApi_Init(void);
void ControllerApi_Reset(void);
void ControllerApi_Step(const ControllerApiInput *input, ControllerApiOutput *output);
void ControllerApi_Terminate(void);

#endif
