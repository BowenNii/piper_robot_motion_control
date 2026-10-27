% - **输入**
%   - `t`：时间向量，n 行 1 列，每个采样点的相对时间（单位：秒）
%   - `q_deg`：关节角度矩阵，n 行 6 列，每列对应一个关节的角度（单位：度）
%   - `cfg`：配置结构体，包含变化阈值、缓冲时长、分段间隔三个参数
% - **输出**
%   - `phase`：n 行 1 列的阶段标记向量，和`t`一一对应：`1=前静态`、`2=运动缓冲段`、`3=后静态`
%   - `bounds`：1×2 向量，原始运动的起止时刻（不含缓冲）
%   - `segments`：识别出的独立运动段数量
%  diff(q_deg,1,1)沿行方向做一阶差分，计算相邻两个采样点的角度差，得到 **(n-1)×6** 的差值矩阵
%  `any(..., 2)`：沿列方向做逻辑或运算
% - 所有采样点默认标记为 `1`（前静态 / 静止状态）；
% - 运动边界默认设为`NaN`（表示无运动）；
% - 独立运动段数默认为 0。

function [phase,bounds,segments]=split_phases(t,q_deg,cfg)
changed=find(any(abs(diff(q_deg,1,1))>cfg.change_threshold_deg,2));
phase=ones(size(t)); bounds=[NaN,NaN]; segments=0;
if isempty(changed), return; end
bounds=[t(changed(1)),t(changed(end)+1)];
segments=1+nnz(diff(t(changed))>cfg.activity_gap_s);
phase(t>=bounds(1)-cfg.motion_padding_s & t<=bounds(2)+cfg.motion_padding_s)=2;
phase(t>bounds(2)+cfg.motion_padding_s)=3;
end

% ### 完整数值示例（直观演示）
% 
% 假设参数：
% 
% - 时间 `t = [0; 0.1; 0.2; 0.3; 0.4; 0.5]`（共 6 个采样点，间隔 0.1 秒）
% - 单关节角度 `q_deg = [0; 0.002; 0.008; 0.01; 0.01; 0.01]`
% - 阈值 `cfg.change_threshold_deg = 0.005`，缓冲 `cfg.motion_padding_s = 0.05`
% 
% 执行流程：
% 
% 1. 差分得到 `[0.002; 0.006; 0.002; 0; 0]`
% 2. 大于阈值的只有第 2 个间隔 → `changed = 2`
% 3. 原始边界 `bounds = [t(2), t(3)] = [0.1, 0.2]`
% 4. 缓冲区间：`0.1-0.05 ~ 0.2+0.05` → `0.05~0.25秒`
% 5. 最终`phase`结果：`[1; 2; 2; 3; 3; 3]`
%    - 阶段 1（前静态）：t=0
%    - 阶段 2（运动缓冲）：t=0.1、0.2
%    - 阶段 3（后静态）：t=0.3、0.4、0.5