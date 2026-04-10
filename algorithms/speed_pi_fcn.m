function iq_ref = speed_pi_fcn(w_ref, w_meas, params)
% Discrete PI speed controller with anti-windup
% params = [Kp, Ki, Ts, output_limit]
persistent int_state
if isempty(int_state)
    int_state = 0;
end
Kp = params(1); Ki = params(2); Ts = params(3); lim = params(4);

err = w_ref - w_meas;
int_state = int_state + Ki * Ts * err;
int_state = max(-lim, min(lim, int_state));
output = Kp * err + int_state;
iq_ref = max(-lim, min(lim, output));
