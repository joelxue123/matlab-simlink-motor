function [w_ref, ui_out] = position_pidreg3_fcn(pos_ref, theta_meas, params)
% Position-loop PIDREG3 controller.
% params = [Kp, Ki_d, Kc, output_limit]

persistent ui out_pre_sat
if isempty(ui)
    ui = 0;
    out_pre_sat = 0;
end

Kp = params(1);
Ki = params(2);
Kc = params(3);
out_lim = params(4);
out_max = out_lim;
out_min = -out_lim;

err = pos_ref - theta_meas;
up = Kp * err;

if Ki == 0
    ui = 0;
    out_pre_sat = up;
    w_ref = min(out_max, max(out_min, out_pre_sat));
    ui_out = 0;
    return;
end

if out_pre_sat > out_max
    if err < 0
        ui = ui + Ki * err;
    else
        ui = ui + Kc * (out_max - out_pre_sat);
    end
elseif out_pre_sat < out_min
    if err > 0
        ui = ui + Ki * err;
    else
        ui = ui + Kc * (out_min - out_pre_sat);
    end
else
    ui = ui + Ki * err;
end

ui = min(out_max, max(out_min, ui));
out_pre_sat = up + ui;
w_ref = min(out_max, max(out_min, out_pre_sat));
ui_out = ui;
end