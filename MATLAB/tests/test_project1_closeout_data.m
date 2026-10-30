function test_project1_closeout_data()
% 当前34段收尾数据/模型与交互图只读回归，不依赖已归档的10-24/10-25实验。
root=setup_piper_paths;repo=fileparts(root);
base=fullfile(repo,'experiments','project1','2026-10-27_movej_repeat','project1_closeout');
D=jsondecode(fileread(fullfile(base,'inventory.json')));
assert(numel(D.records)==34&&numel(unique(D.selected_record_numbers))==30);
R=readtable(fullfile(base,'matlab','run_metrics.csv'));
assert(height(R)==34&&all(R.span_over2_fraction==0));
assert(all(isnan(R.joint_over_fraction)),'未设容差不能写成零超限');
model=make_piper_model;
for k=[1 20 26 30 33 34]
    r=D.records(k);[~,execution_name]=fileparts(r.execution_dir);[~,decoded_name]=fileparts(r.decoded_dir);
    assert(isfolder(fullfile(fileparts(base),r.group,r.run_name,execution_name)));
    f=load_feedback_csv(fullfile(base,'decoded',decoded_name,'feedback_fk.csv'),struct('require_transforms',true,'strict_time',true));
    ix=unique(round(linspace(1,numel(f.t),7)));
    checked=validate_fk_samples(model.S_list,model.M,f.q_rad(ix,:),f.T_cpp(:,:,ix), ...
        struct('position_tolerance_m',1e-9,'rotation_tolerance_rad',1e-7));assert(checked.ok);
end
files=dir(fullfile(base,'matlab','figures','*.fig'));assert(numel(files)==10);
f=openfig(fullfile(base,'matlab','figures','04_joint_angles_outbound.fig'),'new','invisible');
cleanup=onCleanup(@()close(f));
lines=findall(f,'Type','line');assert(any(arrayfun(@(h)numel(h.XData)>1000,lines)));
disp('Project1 closeout data PASS');
end
