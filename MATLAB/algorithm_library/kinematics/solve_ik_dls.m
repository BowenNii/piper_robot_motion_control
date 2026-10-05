function result = solve_ik_dls(S_list,M,target,q_initial,options)
% 与 C++ 同名入口；输入/输出见 solve_ik_poe、ik_options。
if nargin<5 || isempty(options), options=struct(); end
options.method='dls';
result=solve_ik_poe(S_list,M,target,q_initial,options);
end
