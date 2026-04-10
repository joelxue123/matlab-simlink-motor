function [vd_ref, vq_ref] = current_pi_fcn(id_ref, iq_ref, id_meas, iq_meas, omega_e, pi_params, motor_params)
% Discrete PI current controller with cross-coupling decoupling
% pi_params    = [Kp, Ki, Ts, output_limit]
% motor_params = [Ld, Lq, psi_f]
persistent int_d int_q
if isempty(int_d)
    int_d = 0;
    int_q = 0;
end
Kp = pi_params(1); Ki = pi_params(2); Ts = pi_params(3); lim = pi_params(4);
Ld = motor_params(1); Lq = motor_params(2); psi_f = motor_params(3);

% d-axis PI
err_d = id_ref - id_meas;
int_d = int_d + Ki * Ts * err_d;
int_d = max(-lim, min(lim, int_d));
vd_pi = Kp * err_d + int_d;

% q-axis PI
err_q = iq_ref - iq_meas;
int_q = int_q + Ki * Ts * err_q;
int_q = max(-lim, min(lim, int_q));
vq_pi = Kp * err_q + int_q;

% Cross-coupling decoupling
vd_dec = -omega_e * Lq * iq_meas;
vq_dec = omega_e * (Ld * id_meas + psi_f);

% Output
vd_ref = vd_pi + vd_dec;
vq_ref = vq_pi + vq_dec;
