function [tau, state] = controller_pid(cfg, state, ref, measurement)
%CONTROLLER_PID Position PID with measured velocity damping.

Ts = cfg.sim.Ts;
e = ref.q - measurement.q;

state.integral_e = state.integral_e + Ts*e;
state.integral_e = clamp(state.integral_e, -cfg.pid.integral_limit, cfg.pid.integral_limit);

tau = cfg.pid.Kp*e + cfg.pid.Ki*state.integral_e - cfg.pid.Kd*measurement.qdot;
end

function y = clamp(x, lower, upper)
    y = min(max(x, lower), upper);
end
