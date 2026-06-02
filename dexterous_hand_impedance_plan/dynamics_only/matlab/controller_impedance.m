function [tau, state] = controller_impedance(cfg, state, ref, measurement)
%CONTROLLER_IMPEDANCE Joint-space impedance controller.
%
% tau = K*(q_ref - q) + D*(qdot_ref - qdot)

e = ref.q - measurement.q;
edot = ref.qdot - measurement.qdot;

tau = cfg.impedance.K*e + cfg.impedance.D*edot;
end
