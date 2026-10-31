function export_matlab_dynamics_reference(output_path)
% 一次性生成跨语言基准。只读URDF，不连接硬件；已有文件不覆盖。
assert(~isfile(output_path),'已有参考文件不覆盖');
repo=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(fullfile(repo,'MATLAB'));setup_piper_paths;
model=make_piper_model;
rows=zeros(12,78);
for k=1:12
    q=model.q_min+(model.q_max-model.q_min).*(.5+.35*sin((1:6)'*.7+k*.31));
    dq=.3*cos((1:6)'*.3+k*.2);ddq=.4*sin((1:6)'*.5-k*.13);
    tau=rnea(q,dq,ddq,model);g=gravity(q,model);M=crba(q,model);
    F=[.1;-.2;.3;.4;-.5;.6];loaded=rnea(q,dq,ddq,model,F);
    residual=regressor(q,dq,ddq,model)*inertial_parameters(model);
    rows(k,:)=[q',dq',ddq',tau',g',reshape(M',1,[]),loaded',residual'];
end
names=string([compose('q%d',1:6),compose('dq%d',1:6),compose('ddq%d',1:6), ...
    compose('tau%d',1:6),compose('g%d',1:6)]);
for i=1:6,for j=1:6,names(end+1)=sprintf('M%d%d',i,j);end,end
names=[names,compose('loaded%d',1:6),compose('regressor%d',1:6)];
writetable(array2table(rows,'VariableNames',cellstr(names)),output_path);
fprintf('已生成12组MATLAB动力学基准：%s\n',output_path);
end
