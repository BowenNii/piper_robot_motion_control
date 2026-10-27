%TEST_IK_TRAJECTORY_ANALYSIS 真实离线样本复核与损坏CSV拒绝测试。
% 所有修改仅写到临时目录，原实验CSV只读。需先生成默认trajectory.csv。
clear;
matlab_root=fileparts(fileparts(mfilename('fullpath')));
repo=fileparts(matlab_root);
addpath(matlab_root); setup_piper_paths();
input=fullfile(repo,'experiments','project1','2026-10-18_ik_trajectory','data_csv','trajectory.csv');
urdf=fullfile(repo,'models','urdf','piper_description.urdf');
[r,summary,points]=analyze_ik_trajectory(input,urdf);
assert(summary.samples==501 && r.timed && summary.duration_s==10);
assert(height(points)==501 && max(r.position_error_m)<r.config.position_tolerance_m);
folder=tempname; mkdir(folder); path=fullfile(folder,'bad.csv');
D=readtable(input);
bad=removevars(D,'q1_rad'); reject(bad,path,urdf);
bad=D; bad.q1_rad(2)=NaN; reject(bad,path,urdf);
bad=D; bad.index(2)=0; reject(bad,path,urdf);
bad=D; bad.s(2)=0; reject(bad,path,urdf);
bad=D; bad.sample_count(:)=100; reject(bad,path,urdf);
bad=D; bad.rotation_weight(2)=0.3; reject(bad,path,urdf);
bad=D; bad.delta_q1_rad(2)=0.01; reject(bad,path,urdf);
bad=D; bad.q1_rad(2)=10; reject(bad,path,urdf);
bad=D; bad.iterations(2)=-1; reject(bad,path,urdf);
bad=D; bad.weighted_sigma_min(2)=0.5; reject(bad,path,urdf);
bad=D; bad.position_error_m(2)=0.01; reject(bad,path,urdf);
bad=D; bad.target_r11(2)=0; reject(bad,path,urdf);
bad=D; bad.poe_x_m(2)=bad.poe_x_m(2)+0.001; reject(bad,path,urdf);
bad=D; bad.t_s(2)=0; reject(bad,path,urdf);
bad=D; bad.ds_dt(20)=1; reject(bad,path,urdf);
bad=D; bad.dq1_rad_s(20)=0.02; reject(bad,path,urdf);
bad=D; bad.ddq1_rad_s2(20)=0.02; reject(bad,path,urdf);
bad=D; bad.offline_velocity_limit_rad_s(:)=1e-9; reject(bad,path,urdf);
bad=removevars(D,'ddq6_rad_s2'); reject(bad,path,urdf);
fprintf('IK trajectory analysis tests PASS. Temporary outputs: %s\n',folder);

function reject(D,path,urdf)
writetable(D,path);
failed=false;
try, analyze_ik_trajectory(path,urdf); catch, failed=true; end
assert(failed,'损坏的CSV必须被拒绝');
end
