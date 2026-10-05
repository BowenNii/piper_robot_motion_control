function S = make_prismatic_screw_axis(direction)
% 移动方向 -> 6×1 [0;单位方向]；关节量单位 m。
direction=direction(:);
validateattributes(direction,{'numeric'},{'real','finite','numel',3});
assert(norm(direction)>1e-12,'移动方向不能为零');
S=[zeros(3,1);direction/norm(direction)];
end
