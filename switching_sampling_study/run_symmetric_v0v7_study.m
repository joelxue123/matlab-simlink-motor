function result = run_symmetric_v0v7_study(varargin)
%RUN_SYMMETRIC_V0V7_STUDY Study only the symmetric V0/V7 = 50%/50% case.
% This is the simplified entry point for the first stage of the switching
% sampling study. It avoids comparing multiple allocation strategies and
% focuses only on center-aligned PWM timing and sampling-window intuition.

result = run_triangle_carrier_study( ...
    'splitCases', 0.5, ...
    'caseNames', {'Symmetric V0/V7'}, ...
    varargin{:});

end