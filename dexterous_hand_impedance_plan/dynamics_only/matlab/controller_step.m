function [tau, state] = controller_step(controller_name, cfg, state, ref, measurement)
%CONTROLLER_STEP Dispatch one sampled controller update.

switch controller_name
    case 'pid'
        [tau, state] = controller_pid(cfg, state, ref, measurement);
    case 'dob_pd'
        [tau, state] = controller_dob_pd(cfg, state, ref, measurement);
    case 'impedance'
        [tau, state] = controller_impedance(cfg, state, ref, measurement);
    otherwise
        error('Unknown controller: %s', controller_name);
end
end
