function pos_ref = position_scan_ref_fcn(t_now, params)
% Generate a staircase position reference from a scan table.
% params = [start_time, hold_time, points, theta_table...]

start_time = params(1);
hold_time = max(params(2), eps);
points = max(1, round(params(3)));

table = zeros(points, 1);
available = min(points, numel(params) - 3);
if available > 0
    table(1:available) = params(4:3 + available);
end

if t_now < start_time
    pos_ref = table(1);
    return;
end

idx = 1 + floor((t_now - start_time) / hold_time);
if idx < 1
    idx = 1;
elseif idx > points
    idx = points;
end

pos_ref = table(idx);
end