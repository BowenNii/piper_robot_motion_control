% 60秒真实执行数据回归：与summary独立核对、共享时钟对齐、终点目标FK。
root=setup_piper_paths();repo=fileparts(root);
run_dir=fullfile(repo,'experiments','project1','2026-10-24_zero_to_pose','run01');
execution=fullfile(run_dir,'execution_mwd2hy56');
urdf=fullfile(repo,'models','urdf','piper_description.urdf');
[r,j,p,t]=analyze_real_ik_run(run_dir,execution,urdf);
assert(r.alignment=="recorded_monotonic_origin" && r.plan.path_mode==1);
assert(numel(r.plan.t)==3001 && height(r.C)==3001 && r.plan.t(end)==60);
assert(abs(r.offset_s-1.7697158489900175)<1e-12);
assert(abs(max(abs(j.last_minus_goal_rad))-.0011996320581020292)<1e-12);
assert(r.final_last_error_recheck<1e-12 && r.status_bad_count==0);
assert(height(j)==6 && height(t)==6 && all(isfinite(r.p_error_mm)));
assert(r.plan_orientation_change_deg(end)>1,'本轮姿态有变化，不能误标为姿态保持');
m=make_piper_model(urdf);
actual=forward_poe_space(m.S_list,mean(r.q_feedback(r.tail_mask,:),1)',m.M);
desired=forward_poe_space(m.S_list,[.3;.3;-.3;.3;.3;.5],m.M);
[ep,er]=pose_error(actual,desired);
assert(abs(ep*1000-p.endpoint_position_error_mm)<1e-6);
assert(abs(er*180/pi-p.endpoint_orientation_error_deg)<1e-6);
% 验证误差确实按最近发送目标，而不是把两个不同采样率的CSV按行减。
for k=round(linspace(find(r.execution_mask,1),find(r.execution_mask,1,'last'),7))
    index=find(r.C.send_elapsed_s<=r.t_feedback(k),1,'last');
    if isempty(index),index=1;end
    assert(max(abs(r.q_sent_at_feedback(k,:)-r.C{index,cellstr(compose('q%d_rad',1:6))}))<1e-12);
end
fprintf('60秒真机IK数据回归测试通过。\n');
