function sigma = singular_values(J)
% m×n Jacobian -> 降序 min(m,n)×1 奇异值；不补欠驱动方向的零值。
validateattributes(J,{'numeric'},{'real','finite','2d','nonempty'});
sigma=svd(J);
end
