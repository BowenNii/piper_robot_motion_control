% 真机IK记录分析：本轮数值复核，拒绝损坏/错配日志，不连接ROS。
root=setup_piper_paths();repo=fileparts(root);
run_dir=fullfile(repo,'archives','2026-10-24_preliminary','experiments','project1', ...
    '2026-10-24_real_ik_z2mm','run01');
execution_dir=fullfile(run_dir,'execution_2zwqntrv');
urdf=fullfile(repo,'models','urdf','piper_description.urdf');
[r,j,p,t]=analyze_real_ik_run(run_dir,execution_dir,urdf);
assert(r.alignment=="legacy_tail_estimate" && height(j)==6 && height(t)==6);
assert(abs(p.planned_dz_mm-2)<1e-8 && abs(p.planned_dx_mm)<1e-8 && abs(p.planned_dy_mm)<1e-8);
assert(abs(max(abs(j.last_minus_goal_rad))-.0015710336980917783)<1e-12);
assert(r.final_last_error_recheck<1e-12 && r.status_bad_count==0);
assert(r.endpoint_stable && max(r.final_joint_range)<1e-12);
assert(all(isfinite(r.p_feedback),'all') && p.endpoint_position_error_mm>0);
assert(max(abs(r.plan_delta_mm(1,:)))<1e-12);
% 只改测试副本，不动任何真机原始记录。
temp=tempname;mkdir(temp);cleanup=onCleanup(@() rmdir(temp,'s'));
for name={'feedback.csv','commands.csv','status.csv','summary.json'}
    copyfile(fullfile(execution_dir,name{1}),temp);
end
J=r.J;J.q1_rad(3)=NaN;writetable(J,fullfile(temp,'feedback.csv'));
reject=false;try,analyze_real_ik_run(run_dir,temp,urdf);catch,reject=true;end
assert(reject,'应拒绝非有限关节反馈');
copyfile(fullfile(execution_dir,'feedback.csv'),temp);
C=r.C;C.q1_rad(3)=C.q1_rad(3)+.01;writetable(C,fullfile(temp,'commands.csv'));
reject=false;try,analyze_real_ik_run(run_dir,temp,urdf);catch,reject=true;end
assert(reject,'应拒绝实发指令与计划不一致');
copyfile(fullfile(execution_dir,'commands.csv'),temp);
J=r.J;J.receive_elapsed_s(3)=J.receive_elapsed_s(2);writetable(J,fullfile(temp,'feedback.csv'));
reject=false;try,analyze_real_ik_run(run_dir,temp,urdf);catch,reject=true;end
assert(reject,'应拒绝反馈接收时间重复');
% 合成具有明确时钟原点的测试元数据，覆盖新工具日志分支，不冒充历史真实原点。
copyfile(fullfile(execution_dir,'feedback.csv'),temp);
meta=r.execution;meta.trajectory_start_receive_elapsed_s=r.offset_s;
fid=fopen(fullfile(temp,'summary.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(meta));fclose(fid);
[known,~,known_pose]=analyze_real_ik_run(run_dir,temp,urdf);
assert(known.alignment=="recorded_monotonic_origin");
assert(max(abs(known.joint_error-r.joint_error),[],'all')<1e-12);
assert(abs(known_pose.endpoint_position_error_mm-p.endpoint_position_error_mm)<1e-12);
fprintf('真机IK分析：本轮数值复核及三项损坏输入拒绝测试通过。\n');
