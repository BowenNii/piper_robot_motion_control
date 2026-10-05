%% twist_hat 6维Twist →4×4 se3矩阵
function T = twist_hat(V)
validateattributes(V,{'numeric'},{'real','finite','vector','numel',6}); V=V(:);
    T = zeros(4,4);
    w_sk =skew(V(1:3));
    T(1:3,1:3) = w_sk;
    T(1:3,4) = V(4:6);
end
