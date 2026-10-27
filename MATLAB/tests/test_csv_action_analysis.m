% 离线Action记录分析回归：独立数值核对与损坏输入拒绝。
root=setup_piper_paths(); repo=fileparts(root);
source=fullfile(repo,'archives','2026-10-24_preliminary','experiments','project1', ...
    '2026-10-24_offline_csv_action','run01');
reference=fullfile(repo,'experiments','project1','2026-10-18_ik_trajectory','data_csv','trajectory.csv');
[r,joints,timing,execution]=analyze_csv_action_run(source,reference);
assert(height(joints)==6 && height(r.A)==200 && height(r.J)==2058);
assert(abs(max(joints.max_abs_error_rad)-5.879199956870806e-7)<1e-15);
assert(all(joints.final_state_minus_csv_goal_rad==0));
assert(execution.action_success && execution.final_state_error_max_rad==0);
assert(r.error_column_difference<1e-15);
assert(timing.max_ms(1)>120 && timing.max_ms(3)<6);
assert(timing.over_20ms_count(1)==1 && timing.over_20ms_count(3)==0);
assert(abs(timing.median_ms(5)-50)<0.1);

temp=tempname; mkdir(temp); cleanup=onCleanup(@() rmdir(temp,'s'));
copyfile(fullfile(source,'action_feedback.csv'),temp);
copyfile(fullfile(source,'joint_states.csv'),temp);
copyfile(fullfile(source,'summary.json'),temp);
A=r.A; A.error_q1(2)=0.1;
writetable(A,fullfile(temp,'action_feedback.csv'));
rejected=false; try, analyze_csv_action_run(temp,reference); catch, rejected=true; end
assert(rejected,'应拒绝误差列不一致的记录');
A=r.A; A.desired_time_s(2)=A.desired_time_s(1);
writetable(A,fullfile(temp,'action_feedback.csv'));
rejected=false; try, analyze_csv_action_run(temp,reference); catch, rejected=true; end
assert(rejected,'应拒绝重复时间');
copyfile(fullfile(source,'action_feedback.csv'),temp);
J=r.J; J.joint3(5)=NaN; writetable(J,fullfile(temp,'joint_states.csv'));
rejected=false; try, analyze_csv_action_run(temp,reference); catch, rejected=true; end
assert(rejected,'应拒绝非有限关节角');
fprintf('CSV Action分析：真实记录数值核对与三项异常输入拒绝测试通过。\n');
