function value = rad_to_deg(angle)
% 角度数组 -> 相同尺寸转换结果。
validateattributes(angle,{'numeric'},{'real','finite'});
value=angle*180/pi;
end
