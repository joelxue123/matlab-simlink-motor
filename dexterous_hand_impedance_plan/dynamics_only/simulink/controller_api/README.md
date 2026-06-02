# Controller API Wrapper

This wrapper makes the generated Simulink controller code easier to call from STM32 firmware.

Generated code:

```text
../Controller_grt_rtw/usr_pid.c
../Controller_grt_rtw/usr_pid.h
```

Wrapper:

```text
controller_api.c
controller_api.h
```

## STM32-style usage

```c
#include "controller_api.h"

static ControllerApiInput ctrl_in;
static ControllerApiOutput ctrl_out;

void app_init(void)
{
  ControllerApi_Init();
}

void control_loop_1khz(void)
{
  ctrl_in.q_ref = q_ref;
  ctrl_in.q = q_meas;
  ctrl_in.qdot = qdot_meas;
  ctrl_in.qddot = qddot_est;
  ctrl_in.tau_prev = tau_prev;
  ctrl_in.mode = 2U;  /* 1=PID, 2=DOB+PD, 3=Impedance */

  ControllerApi_Step(&ctrl_in, &ctrl_out);

  motor_set_torque(ctrl_out.tau_cmd);
  tau_prev = ctrl_out.tau_cmd;
}
```

## Notes

The generated reusable function uses fixed-point integer ports:

```text
q_ref, q      : int16, fixdt(1,16,12)
qdot          : int16, fixdt(1,16,8)
qddot         : int16, fixdt(1,16,4)
tau_prev      : int16, fixdt(1,16,12)
tau_cmd       : int16, fixdt(1,16,13)
tau_load_hat  : int16, fixdt(1,16,13)
mode          : uint8
```

Convert physical units to raw fixed-point values before calling `ControllerApi_Step`, and convert the output raw torque command back to physical units or to the motor driver scale after the call.
