function out = simulate_single_joint(cfg, controller_name, test_name, delay_samples)
%SIMULATE_SINGLE_JOINT Simulate one sampled controller on one joint.
%
% Plant:
%   J*qddot + b*qdot = tau_applied - tau_load

Ts = cfg.sim.Ts;
t = (0:Ts:cfg.sim.T_end).';
n = numel(t);

q = zeros(n, 1);
qdot = zeros(n, 1);
qddot = zeros(n, 1);
q_ref = zeros(n, 1);
qdot_ref = zeros(n, 1);
tau_cmd = zeros(n, 1);
tau_applied = zeros(n, 1);
tau_load = zeros(n, 1);
current = zeros(n, 1);
heat_power = zeros(n, 1);
tau_load_hat = zeros(n, 1);

q(1) = cfg.sim.q0;
qdot(1) = cfg.sim.qdot0;

state = init_controller_state(controller_name);

if delay_samples > 0
    delay_buffer = zeros(delay_samples, 1);
else
    delay_buffer = [];
end

prev_qdot = qdot(1);
prev_tau_applied = 0.0;

for k = 1:n
    [q_ref(k), qdot_ref(k)] = reference_signal(t(k), cfg, test_name);
    tau_load(k) = load_torque(t(k), cfg, test_name);

    measurement.q = q(k);
    measurement.qdot = qdot(k);
    measurement.qddot = (qdot(k) - prev_qdot) / Ts;
    measurement.prev_tau_applied = prev_tau_applied;

    ref.q = q_ref(k);
    ref.qdot = qdot_ref(k);

    [tau_cmd(k), state] = controller_step(controller_name, cfg, state, ref, measurement);
    tau_cmd(k) = clamp(tau_cmd(k), -cfg.sim.tau_limit, cfg.sim.tau_limit);

    if delay_samples == 0
        tau_applied(k) = tau_cmd(k);
    else
        tau_applied(k) = delay_buffer(1);
        delay_buffer = [delay_buffer(2:end); tau_cmd(k)]; %#ok<AGROW>
    end

    current(k) = tau_applied(k) / cfg.plant.Kt;
    heat_power(k) = current(k)^2 * cfg.plant.R;
    tau_load_hat(k) = get_load_hat(controller_name, state);

    if k < n
        qddot(k) = (tau_applied(k) - tau_load(k) - cfg.plant.b*qdot(k)) / cfg.plant.J;
        qdot(k + 1) = qdot(k) + Ts*qddot(k);
        q(k + 1) = q(k) + Ts*qdot(k + 1);

        prev_qdot = qdot(k);
        prev_tau_applied = tau_applied(k);
    end
end

qddot(n) = (tau_applied(n) - tau_load(n) - cfg.plant.b*qdot(n)) / cfg.plant.J;

out = struct();
out.controller = controller_name;
out.test = test_name;
out.delay_samples = delay_samples;
out.t = t;
out.q = q;
out.qdot = qdot;
out.qddot = qddot;
out.q_ref = q_ref;
out.qdot_ref = qdot_ref;
out.e = q_ref - q;
out.tau_cmd = tau_cmd;
out.tau_applied = tau_applied;
out.tau_load = tau_load;
out.tau_load_hat = tau_load_hat;
out.current = current;
out.heat_power = heat_power;
end

function state = init_controller_state(controller_name)
    state = struct();
    state.integral_e = 0.0;
    state.tau_load_hat = 0.0;
    state.controller_name = controller_name;
end

function [q_ref, qdot_ref] = reference_signal(t, cfg, test_name)
    %#ok<INUSD>
    if t < cfg.ref.step_time
        q_ref = cfg.ref.q_initial;
    else
        q_ref = cfg.ref.q_final;
    end
    qdot_ref = 0.0;
end

function tau_load = load_torque(t, cfg, test_name)
    switch test_name
        case 'load_step'
            if t >= cfg.load.step_time
                tau_load = cfg.load.tau_value;
            else
                tau_load = 0.0;
            end
        otherwise
            tau_load = 0.0;
    end
end

function tau_hat = get_load_hat(controller_name, state)
    if strcmp(controller_name, 'dob_pd')
        tau_hat = state.tau_load_hat;
    else
        tau_hat = 0.0;
    end
end

function y = clamp(x, lower, upper)
    y = min(max(x, lower), upper);
end
