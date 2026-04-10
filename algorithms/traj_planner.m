function pos_out = traj_planner(pos_cmd, params)
% 梯形速度规划器
% params = [max_vel, max_acc, Ts]
persistent pos vel
if isempty(pos)
    pos = 0; vel = 0;
end
v_max = params(1); a_max = params(2); Ts = params(3);

err = pos_cmd - pos;
% 减速距离: d_dec = v^2 / (2*a)
d_dec = vel * abs(vel) / (2 * a_max);

if abs(err) > abs(d_dec) + 1e-6
    % 加速/匀速阶段
    vel = vel + sign(err) * a_max * Ts;
    vel = max(-v_max, min(v_max, vel));
else
    % 减速阶段
    vel = vel - sign(vel) * a_max * Ts;
    if sign(vel) ~= sign(err) && abs(err) < 1e-4
        vel = 0;
    end
end
pos = pos + vel * Ts;
pos_out = pos;
