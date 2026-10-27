function transforms=cpp_columns_to_transforms(position_m,rotation_rows)
%把 CSV 里摊平的位姿数据，还原成带时间维度的 4×4 齐次变换矩阵序列，同时逐点校验每个矩阵的数学合法性。
% 输入：position_m N×3；rotation_rows N×9，按r11,r12,r13,r21,...,r33。
% 输出：4×4×N；reshape按列填充，转置后恢复CSV按行顺序。
validateattributes(position_m,{'numeric'},{'real','finite','2d','nonempty'});
validateattributes(rotation_rows,{'numeric'},{'real','finite','2d','nonempty'});
%MATLAB 内置校验工具，检查输入必须是数值型、实数、有限值、二维矩阵（行+列两个维度）、非空
%排除字符串、空数组、NaN/Inf 等无效输入。
assert(size(position_m,2)==3 && size(rotation_rows,2)==9 ...
    && size(position_m,1)==size(rotation_rows,1),'输入必须N×3和N×9');
N=size(position_m,1);
%把 4×4 的单位矩阵，沿着**第 3 维度（时间维度）**复制 N 份
%最终得到一个 `4×4×N` 的三维数组。
transforms=repmat(eye(4),1,1,N);
for k=1:N
    transforms(1:3,1:3,k)=reshape(rotation_rows(k,:),3,3)';
    transforms(1:3,4,k)=position_m(k,:)';
    report=check_se3(transforms(:,:,k));
    assert(report.ok,'第%d行C++位姿非法：%s',k,report.message);
end
end
% % 2个采样点的位置
% position_m = [0.1, 0.2, 0.3;  0.4, 0.5, 0.6];
% % 2个采样点的旋转矩阵（按行展开，这里用单位矩阵举例）
% rotation_rows = [1 0 0 0 1 0 0 0 1;  1 0 0 0 1 0 0 0 1];
% % 第1页（第1个时刻）
% transforms(:,:,1) = 
% [1  0  0  0.1;
%  0  1  0  0.2;
%  0  0  1  0.3;
%  0  0  0  1  ]
% 
% % 第2页（第2个时刻）
% transforms(:,:,2) = 
% [1  0  0  0.4;
%  0  1  0  0.5;
%  0  0  1  0.6;
%  0  0  0  1  ]

