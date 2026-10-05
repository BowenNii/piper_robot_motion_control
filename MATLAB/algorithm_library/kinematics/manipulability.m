function value = manipulability(J)
% 与当前 C++ 实现对齐：奇异值乘积。
% m<=n 时等于 sqrt(det(J*J'))；m>n 时只是非零子空间体积，
% 不能代表完整任务空间可操作度（完整任务空间的体积为0）。
value=prod(singular_values(J));
end
