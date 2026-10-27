function data = load_feedback_csv(path,cfg)
% 输入：C++ 从 CAN 日志解析后导出的原始扁平 CSV：列多、命名零散、单位混杂（弧度、米、秒）
% 没有结构化，直接拿来写分析逻辑会非常繁琐，供后续所有分析、计算、绘图函数直接调用。
% 1. 多层质检：列完整性、时间单调性、数值合法性逐层校验，脏数据直接拦在入口；
% 2. 格式标准化：把零散的列整理成统一命名的字段，统一转换成常用单位（mm、°、ms），统一生成从 0 开始的相对时间；
% 3. 按需扩展：基础模式只加载统计必需的字段，速度快；开启完整模式时额外生成 4×4×N 的三维位姿矩阵，供交叉验证使用。
% 输出：一个 MATLAB `struct` 结构体，相当于一套内部统一的数据标准
% - 原始表格完整保留可溯源；
% - 常用字段（时间、角度、位置、误差）直接用 `.` 就能访问，不用再记 CSV 原始列名；
% - 需要矩阵运算时，`.T_can`、`.T_cpp` 直接就是标准的三维变换矩阵序列。

% cfg.require_transforms=false：静态统计只需角度、XYZ和误差列，默认精简模式，不生成完整的 4×4 变换矩阵；
% true时额外要求CAN RPY及C++旋转矩阵（用于独立交叉验证）。
% cfg.strict_time=false允许重复时刻但不允许倒退；动态实验设true。
% 输出：table原表、source、t相对秒、timestamp_s、q_rad/q_deg、
% p_feedback/p_cpp米、position_mm、rotation_deg、span_ms、timing质量报告。
% 完整位姿模式额外提供T_can/T_cpp（4×4×N）及rpy_feedback（rad）。
% 格式错误直接停止；跨度/间隔异常只报告，主脚本决定如何处理。
if nargin<2, cfg=struct; end%如果只传了文件路径path，没传cfg，就先创建一个空的结构体
if ~isfield(cfg,'require_transforms'), cfg.require_transforms=false; end
if ~isfield(cfg,'strict_time'), cfg.strict_time=false; end
validateattributes(cfg.require_transforms,{'logical'},{'scalar'});%必须是逻辑型标量
validateattributes(cfg.strict_time,{'logical'},{'scalar'});
assert(isfile(path),'找不到数据文件：%s',path);
D=readtable(path,'VariableNamingRule','preserve');
q_cols=cellstr(compose('q%d_rad',1:6));
required=[{'t_s','timestamp_s','frame_span_s'},q_cols, ...
    {'feedback_x_m','feedback_y_m','feedback_z_m','poe_x_m','poe_y_m','poe_z_m', ...
    'position_error_m','rotation_error_rad'}];
if cfg.require_transforms%true时需要生成完整 4×4 变换矩阵
    r_cols={'poe_r11','poe_r12','poe_r13','poe_r21','poe_r22','poe_r23', ...
        'poe_r31','poe_r32','poe_r33'};
    required=[required,{'feedback_roll_rad','feedback_pitch_rad','feedback_yaw_rad'},r_cols];
end
report=check_feedback_data(D,required);%逐列检查表格里有没有对应列、列内数据是不是合法数值
assert(report.ok,'%s：%s',path,strjoin(report.messages,'；'));
assert(all(diff(D.t_s)>=0) && all(diff(D.timestamp_s)>=0),'时间不能倒退');
if cfg.strict_time
    assert(height(D)>=2 && all(diff(D.t_s)>0) && all(diff(D.timestamp_s)>0), ...
        '动态实验至少两条样本，时间必须严格递增');
end
assert(max(abs((D.t_s-D.t_s(1))-(D.timestamp_s-D.timestamp_s(1))))<1e-6, ...
    '相对时间与绝对时间戳不一致');
assert(all(D.frame_span_s>=0) && all(D.position_error_m>=0) ...
    && all(D.rotation_error_rad>=0 & D.rotation_error_rad<=pi), ...
    '跨度或误差列范围不合法');
data = struct(...
    'table', D, ...                  % 1. 完整保留原始表格
    'source', string(path), ...      % 2. 记录源文件路径
    't', D.t_s - D.t_s(1), ...      % 3. 从0开始的相对时间
    'timestamp_s', D.timestamp_s, ...% 4. 绝对时间戳
    'q_rad', D{:,q_cols}, ...        % 5. 关节角度（弧度）
    'p_feedback', D{:,{'feedback_x_m','feedback_y_m','feedback_z_m'}}, ...  % 6. 硬件反馈位置
    'p_cpp', D{:,{'poe_x_m','poe_y_m','poe_z_m'}}, ...        % 7. C++计算位置
    'position_mm', D.position_error_m*1000, ...  % 8. 位置误差（转mm）
    'rotation_deg', D.rotation_error_rad*180/pi, ... % 9. 姿态误差（转°）
    'span_ms', D.frame_span_s*1000 ... % 10. 帧跨度（转ms）
);
data.q_deg=data.q_rad*180/pi;%生成角度版的关节角度
data.timing=check_sample_timing(data.t,D.frame_span_s,cfg);%调用时序质量检查函数，生成采样间隔、丢包、卡顿等统计报告
if cfg.require_transforms%完整位姿矩阵生成
    data.rpy_feedback=D{:,{'feedback_roll_rad','feedback_pitch_rad','feedback_yaw_rad'}};
    data.T_can=feedback_rpy_to_transforms(data.p_feedback,data.rpy_feedback);
    data.T_cpp=cpp_columns_to_transforms(data.p_cpp,D{:,r_cols});
end
end
