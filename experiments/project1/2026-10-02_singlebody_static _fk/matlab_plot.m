clear; clc; close all;

%% 1. 读取 C++ 导出的 CSV
% 请打开并运行这个 .m 文件；不要复制到命令窗口，否则无法确定脚本位置。
script_dir = fileparts(mfilename('fullpath'));
csv_path = fullfile(script_dir, 'data.csv');
if ~isfile(csv_path)
    error('找不到实验数据文件：%s', csv_path);
end
data = readtable(csv_path, 'Delimiter', ',');

t = data.t_s;                        % 相对时间，单位 s
q_rad = data{:, {'q1_rad', 'q2_rad', 'q3_rad', ...
                'q4_rad', 'q5_rad', 'q6_rad'}};

p_feedback = data{:, {'feedback_x_m', ...
                     'feedback_y_m', ...
                     'feedback_z_m'}};

p_poe = data{:, {'poe_x_m', 'poe_y_m', 'poe_z_m'}};

% 仅在显示时换单位，不改变原始数据
q_deg = q_rad * 180 / pi;
p_feedback_mm = p_feedback * 1000;
p_poe_mm = p_poe * 1000;
position_error_mm = data.position_error_m * 1000;
rotation_error_deg = data.rotation_error_rad * 180 / pi;

%% 2. 六个关节角度随时间变化
figure('Name', '关节角度', 'Color', 'w');
tiledlayout(3, 2);

for i = 1:6
    nexttile;
    plot(t, q_deg(:, i), 'LineWidth', 1.2);
    grid on;
    xlabel('时间 / s');
    ylabel('角度 / °');
    title(sprintf('J%d', i));
end

sgtitle('六关节角度反馈');

%% 3. 末端 XYZ：厂家反馈与 POE 计算对比
figure('Name', '末端位置对比', 'Color', 'w');
tiledlayout(3, 1);

axis_names = {'X', 'Y', 'Z'};

for i = 1:3
    nexttile;
    plot(t, p_feedback_mm(:, i), 'b-', 'LineWidth', 1.2);
    hold on;
    plot(t, p_poe_mm(:, i), 'r--', 'LineWidth', 1.2);
    grid on;
    xlabel('时间 / s');
    ylabel(sprintf('%s / mm', axis_names{i}));
    legend('厂家反馈', 'POE计算', 'Location', 'best');
end

sgtitle('末端位置：反馈与模型对比');

%% 4. 位置误差与姿态误差
figure('Name', '模型一致性误差', 'Color', 'w');
tiledlayout(2, 1);

nexttile;
plot(t, position_error_mm, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('位置误差 / mm');
title('位置误差：两位置向量之差的模');

nexttile;
plot(t, rotation_error_deg, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('姿态误差 / °');
title('姿态误差：相对旋转矩阵的转角');

%% 5. 输出误差统计
fprintf('有效数据组数：%d\n', height(data));

fprintf('位置误差：均值 %.6f mm，RMS %.6f mm，最大 %.6f mm\n', ...
    mean(position_error_mm), ...
    sqrt(mean(position_error_mm.^2)), ...
    max(position_error_mm));

fprintf('姿态误差：均值 %.6f°，RMS %.6f°，最大 %.6f°\n', ...
    mean(rotation_error_deg), ...
    sqrt(mean(rotation_error_deg.^2)), ...
    max(rotation_error_deg));
