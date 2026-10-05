function value = deg_to_rad(angle)
% 角度数组 -> 相同尺寸转换结果。
validateattributes(angle,{'numeric'},{'real','finite'});
value=angle*pi/180;
end
