function compare_s_z_models(Ts, poleZ)
%COMPARE_S_Z_MODELS Compare continuous s-domain and discrete z-domain models.
%   COMPARE_S_Z_MODELS() uses Ts = 1/10000 and poleZ = 0.95.
%   COMPARE_S_Z_MODELS(TS, POLEZ) lets you override both values.

if nargin < 1 || isempty(Ts)
    Ts = 1 / 10000;
end

if nargin < 2 || isempty(poleZ)
    poleZ = 0.95;
end

if Ts <= 0
    error('Ts must be positive.');
end

if abs(poleZ) >= 1
    warning('poleZ is on or outside the unit circle; continuous approximation may not be meaningful.');
end

tau = -Ts / log(poleZ);

Hz = tf([1 -1], [1 -poleZ], Ts, 'Variable', 'z^-1');
Hs = tf([tau 0], [tau 1]);

w = logspace(0, log10(pi / Ts), 1200);

[magS, phaseS] = bode(Hs, w);
[magZ, phaseZ] = bode(Hz, w);
magS = squeeze(magS);
magZ = squeeze(magZ);
phaseS = squeeze(phaseS);
phaseZ = squeeze(phaseZ);

tFinal = max(8 * tau, 100 * Ts);
tCont = linspace(0, tFinal, 1200);
tDisc = 0:Ts:tFinal;

[yStepS, tStepS] = step(Hs, tCont);
[yStepZ, tStepZ] = step(Hz, tDisc);
[yImpS, tImpS] = impulse(Hs, tCont);
[yImpZ, tImpZ] = impulse(Hz, tDisc);

fprintf('\nS/Z comparison\n');
fprintf('Ts      = %.9g s\n', Ts);
fprintf('poleZ   = %.9g\n', poleZ);
fprintf('tau     = %.9g s\n', tau);
fprintf('Hs(s)   = (tau*s)/(1 + tau*s)\n');
fprintf('Hz(z)   = (1 - z^-1)/(1 - %.9g z^-1)\n', poleZ);
fprintf('Nyquist = %.3f Hz\n', 1 / (2 * Ts));

figure('Name', 'S vs Z comparison', 'Color', 'w');

subplot(2, 2, 1);
semilogx(w / (2 * pi), 20 * log10(magS), 'LineWidth', 1.3);
hold on;
semilogx(w / (2 * pi), 20 * log10(magZ), '--', 'LineWidth', 1.3);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Bode magnitude');
legend('S-domain', 'Z-domain', 'Location', 'best');

subplot(2, 2, 2);
semilogx(w / (2 * pi), phaseS, 'LineWidth', 1.3);
hold on;
semilogx(w / (2 * pi), phaseZ, '--', 'LineWidth', 1.3);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
title('Bode phase');
legend('S-domain', 'Z-domain', 'Location', 'best');

subplot(2, 2, 3);
plot(tStepS, squeeze(yStepS), 'LineWidth', 1.3);
hold on;
stairs(tStepZ, squeeze(yStepZ), '--', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Step response');
legend('S-domain', 'Z-domain', 'Location', 'best');

subplot(2, 2, 4);
plot(tImpS, squeeze(yImpS), 'LineWidth', 1.3);
hold on;
stairs(tImpZ, squeeze(yImpZ), '--', 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Amplitude');
title('Impulse response');
legend('S-domain', 'Z-domain', 'Location', 'best');

sgtitle(sprintf('Compare S and Z models, Ts = %.6g s, pole = %.4f', Ts, poleZ));
end