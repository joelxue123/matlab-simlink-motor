function theta_cum = pos_unwrap(mtr_pos)
% Unwrap MtrPos [0,2pi) to cumulative angle
persistent prev_pos revs
if isempty(prev_pos)
    prev_pos = mtr_pos;
    revs = 0;
end
delta = mtr_pos - prev_pos;
if delta < -pi        % 正转过零: 0->2pi 跳到 2pi->0
    revs = revs + 1;
elseif delta > pi     % 反转过零
    revs = revs - 1;
end
prev_pos = mtr_pos;
theta_cum = mtr_pos + revs * 2 * pi;
