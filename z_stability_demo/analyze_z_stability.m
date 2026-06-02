function result = analyze_z_stability(num, den, Ts, varargin)
%ANALYZE_Z_STABILITY Analyze discrete-time z-domain stability and margins.
%   RESULT = ANALYZE_Z_STABILITY(NUM, DEN, TS) evaluates the discrete-time
%   transfer function H(z) = NUM(z^-1) / DEN(z^-1), where NUM and DEN are
%   coefficient vectors in descending powers of z^-1. TS is the sample time.
%
%   RESULT contains poles, zeros, stability verdict, frequency response,
%   gain crossover, phase crossover, phase margin, and gain margin.

if nargin < 3
    error('analyze_z_stability requires num, den, and Ts.');
end

if ~isscalar(Ts) || Ts <= 0
    error('Ts must be a positive scalar.');
end

num = num(:).';
den = den(:).';

if isempty(num) || isempty(den)
    error('num and den must be non-empty coefficient vectors.');
end

if den(1) == 0
    error('Leading denominator coefficient must be non-zero.');
end

opt.nPoints = 4096;
if ~isempty(varargin)
    for k = 1:2:numel(varargin)
        opt.(varargin{k}) = varargin{k + 1};
    end
end

num = num ./ den(1);
den = den ./ den(1);

zZeros = roots(num);
zPoles = roots(den);
poleRadius = abs(zPoles);
tol = 1e-9;

isStable = all(poleRadius < 1 - tol);
isMarginal = ~isStable && all(poleRadius <= 1 + tol);

Omega = linspace(1e-6, pi, opt.nPoints);
ejOmega = exp(-1j * Omega);
numResp = polyval(num, ejOmega);
denResp = polyval(den, ejOmega);
H = numResp ./ denResp;

mag = abs(H);
magDb = 20 * log10(mag);
phaseDeg = unwrap(angle(H)) * 180 / pi;
omega = Omega / Ts;

[gainCrossOmega, phaseMargin] = localPhaseMargin(Omega, magDb, phaseDeg);
[phaseCrossOmega, gainMarginDb] = localGainMargin(Omega, magDb, phaseDeg);

if isnan(gainCrossOmega)
    gainCrossHz = NaN;
else
    gainCrossHz = gainCrossOmega / (2 * pi * Ts);
end

if isnan(phaseCrossOmega)
    phaseCrossHz = NaN;
else
    phaseCrossHz = phaseCrossOmega / (2 * pi * Ts);
end

result = struct();
result.num = num;
result.den = den;
result.Ts = Ts;
result.zeros = zZeros;
result.poles = zPoles;
result.poleRadius = poleRadius;
result.isStable = isStable;
result.isMarginal = isMarginal;
result.frequency_rad_s = omega;
result.frequency_rad_sample = Omega;
result.response = H;
result.magnitude = mag;
result.magnitude_db = magDb;
result.phase_deg = phaseDeg;
result.gain_crossover_rad_sample = gainCrossOmega;
result.gain_crossover_hz = gainCrossHz;
result.phase_margin_deg = phaseMargin;
result.phase_crossover_rad_sample = phaseCrossOmega;
result.phase_crossover_hz = phaseCrossHz;
result.gain_margin_db = gainMarginDb;

fprintf('\nDiscrete transfer function analysis\n');
fprintf('Ts = %.9g s\n', Ts);
fprintf('Numerator  (z^-1 form): [%s]\n', num2str(num));
fprintf('Denominator(z^-1 form): [%s]\n', num2str(den));
fprintf('Poles:\n');
disp(zPoles);
fprintf('Pole radii:\n');
disp(poleRadius);

if isStable
    fprintf('Verdict: stable (all poles strictly inside unit circle).\n');
elseif isMarginal
    fprintf('Verdict: marginal (at least one pole on unit circle).\n');
else
    fprintf('Verdict: unstable (at least one pole outside unit circle).\n');
end

if isnan(phaseMargin)
    fprintf('Phase margin: not found (no 0 dB crossover in scanned range).\n');
else
    fprintf('Phase margin: %.3f deg at %.3f Hz\n', phaseMargin, gainCrossHz);
end

if isnan(gainMarginDb)
    fprintf('Gain margin: not found (no -180 deg crossover in scanned range).\n');
else
    fprintf('Gain margin: %.3f dB at %.3f Hz\n', gainMarginDb, phaseCrossHz);
end
end

function [OmegaCross, phaseMargin] = localPhaseMargin(Omega, magDb, phaseDeg)
signVec = sign(magDb);
crossIdx = find(signVec(1:end-1) .* signVec(2:end) <= 0, 1, 'first');

if isempty(crossIdx)
    OmegaCross = NaN;
    phaseMargin = NaN;
    return;
end

OmegaCross = interp1(magDb(crossIdx:crossIdx+1), Omega(crossIdx:crossIdx+1), 0);
phaseAtCross = interp1(Omega(crossIdx:crossIdx+1), phaseDeg(crossIdx:crossIdx+1), OmegaCross);
phaseMargin = 180 + phaseAtCross;
end

function [OmegaCross, gainMarginDb] = localGainMargin(Omega, magDb, phaseDeg)
shiftedPhase = phaseDeg + 180;
signVec = sign(shiftedPhase);
crossIdx = find(signVec(1:end-1) .* signVec(2:end) <= 0, 1, 'first');

if isempty(crossIdx)
    OmegaCross = NaN;
    gainMarginDb = NaN;
    return;
end

OmegaCross = interp1(shiftedPhase(crossIdx:crossIdx+1), Omega(crossIdx:crossIdx+1), 0);
magAtCross = interp1(Omega(crossIdx:crossIdx+1), magDb(crossIdx:crossIdx+1), OmegaCross);
gainMarginDb = -magAtCross;
end