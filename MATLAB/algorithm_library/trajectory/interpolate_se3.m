function T = interpolate_se3(T0,T1,s)
% 两个 4×4 位姿、比例 s -> 位姿；s 限制到 [0,1]。
validateattributes(s,{'numeric'},{'real','finite','scalar'});
T=T0*se3_exp(clamp(s,0,1)*se3_log(se3_inverse(T0)*T1));
end
