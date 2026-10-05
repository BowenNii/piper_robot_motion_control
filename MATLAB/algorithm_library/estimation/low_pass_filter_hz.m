function output = low_pass_filter_hz(input,previous_output,dt,cutoff_hz)
% 周期 s、截止频率 Hz -> 低通输出；扩展接口，标准接口使用 alpha。
output=low_pass_filter(input,previous_output,dt,cutoff_hz);
end
