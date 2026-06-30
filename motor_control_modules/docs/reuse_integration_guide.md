# Reuse Integration Guide

This guide explains how another project should consume the motor-control MBD
modules.

## Simulink-Level Reuse

Use this flow when teammates are building or extending Simulink models:

```text
1. Run motor_control_modules/setup_motor_control_modules.m.
2. Run motor_control_modules/build_motor_control_interface_dictionary.m.
3. Run motor_control_modules/build_motor_control_module_library.m.
4. Run sl_refresh_customizations if the Library Browser is already open.
5. Open the Simulink Library Browser and find Motor Control Modules.
6. Drag linked library blocks into the integration model.
7. Attach the required .sldd contracts.
8. Add Rate Transition blocks at every multi-rate boundary.
9. Run the module smoke tests before changing internals.
```

Do not copy subsystem contents from old demo models. Copying breaks update
tracking and creates hidden forks.

## Copying From The Library Window

If dragging from the opened `.slx` library window is inconvenient, copy/paste is
acceptable when the source is:

```text
motor_control_modules/motor_control_lib.slx
```

This is different from copying subsystem contents out of a demo model. A block
copied from a Simulink library should remain a linked library block.

After pasting into the target model, select the pasted block and check:

```matlab
get_param(gcb, 'LinkStatus')
get_param(gcb, 'ReferenceBlock')
```

Expected result:

```text
LinkStatus     = resolved
ReferenceBlock = motor_control_lib/CurrentPiStep
```

The exact block name can be `SpeedPiStep`, `CurrentPiStep`,
`DqToAbcDutyStep`, `MotorClarkeParkStep`, or `OpenLoopCommand`.

If `LinkStatus` is `none` or the block has no `ReferenceBlock`, the block has
become an independent copy and should not be used for team reuse.

## C-Level Reuse

Use this flow when firmware teammates need embedded C:

```text
1. Generate controller-only C for the required modules.
2. Keep generated C platform independent.
3. Write a hand-owned platform adapter for ADC/PWM/encoder/timer.
4. Convert raw hardware values to physical input buses.
5. Call generated step functions from the scheduler.
6. Convert duty output buses to PWM compare values.
```

Recommended boundary:

```text
generated controller core:
  no registers
  no HAL
  no chip headers

platform adapter:
  ADC
  PWM
  encoder
  timer
  DMA
  interrupt
```

## Scheduling Pattern

Current baseline:

```text
25us PWM tick
50us current loop
100us speed loop
```

For PWM dead-time compensation, the reuse pattern is:

```text
motor_control_lib/DeadtimeCompensationStep
```

Feed it with a `pwm_deadtime_comp_input_t` bus and use the
`pwm_deadtime_comp_output_t` bus in the PWM adapter. Do not paste the
algorithm contents into each plant model.

Example platform-neutral schedule:

```c
void MotorControl_25usTick(void)
{
    read_platform_inputs();

    if ((tick % 4U) == 0U) {
        run_speed_loop_100us();
    }

    if ((tick % 2U) == 0U) {
        run_current_loop_50us();
        run_dq_to_duty_50us();
    }

    write_platform_pwm(last_phase_duty);
    tick++;
}
```

## Adapter Examples

The same controller core should be callable from:

```text
platform/ti_c2000_adapter/
platform/st_stm32_adapter/
platform/nxp_adapter/
platform/autosar_adapter/
platform/linux_sil_adapter/
```

These adapters are allowed to know about registers or vendor HAL. The
controller core is not.
