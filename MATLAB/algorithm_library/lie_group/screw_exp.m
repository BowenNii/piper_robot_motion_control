function T = screw_exp(S,theta)
% 旋量轴 6×1、关节量 rad 或 m -> 4×4 exp([S]*theta)。
validateattributes(S,{'numeric'},{'real','finite','vector','numel',6});
validateattributes(theta,{'numeric'},{'real','finite','scalar'});
T=se3_exp(S(:)*theta);
end
