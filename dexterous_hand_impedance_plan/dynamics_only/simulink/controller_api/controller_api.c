#include "controller_api.h"

static B_usr_pid_T controller_local_b;
static DW_usr_pid_T controller_local_dw;

void ControllerApi_Init(void)
{
  ControllerApi_Reset();
}

void ControllerApi_Reset(void)
{
  usr_pid_Init(&controller_local_dw);
  controller_local_b.tau_cmd = 0;
  controller_local_b.tau_load_hat = 0;
}

void ControllerApi_Step(const ControllerApiInput *input, ControllerApiOutput *output)
{
  usr_pid(input->q_ref,
          input->q,
          input->qdot,
          input->qddot,
          input->tau_prev,
          input->mode,
          &controller_local_b,
          &controller_local_dw);

  output->tau_cmd = controller_local_b.tau_cmd;
  output->tau_load_hat = controller_local_b.tau_load_hat;
}

void ControllerApi_Terminate(void)
{
}
