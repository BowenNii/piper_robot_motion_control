%TEST_PLOTTING 用人工数据验证绘图、单位换算和保存；不动实验结果。
clear;
addpath(fullfile(fileparts(mfilename('fullpath')),'..'));
setup_piper_paths;
folder=tempname; mkdir(folder);
cfg=struct('visible','off','title','绘图模块测试','bounds_s',[.2,.8]);
t=(0:.1:1)'; q=repmat(t,1,6)*pi/6;
figs=gobjects(0);
try
    [f,ax]=plot_joint_angles(t,q,cfg); figs(end+1)=f;
    assert(numel(ax)==6);
    line=findobj(ax(1),'Type','line');
    assert(abs(line.YData(end)-30)<1e-12);
    path=save_experiment_figure(f,folder,'matlab_关节角度.png');
    assert(isfile(path)); expect_error(@()save_experiment_figure(f,folder,'matlab_关节角度.png'));
    save_experiment_figure(f,folder,'matlab_关节角度.png',struct('overwrite',true));
    save_experiment_figure(f,folder,'editable.fig');
    p=[t,t*0,t*2]/1000;
    [f,ax]=plot_position_comparison(t,{p,p*1.1},{'CAN反馈','MATLAB POE'},cfg);
    figs(end+1)=f; assert(numel(ax)==3);
    h=findobj(ax(1),'Type','line'); assert(numel(h)==2);
    assert(numel(findall(f,'Type','legend'))==3);
    save_experiment_figure(f,folder,'matlab_XYZ.png');
    [f,ax]=plot_pose_errors(t,t*.001,t*pi/180,cfg); figs(end+1)=f;
    h=findobj(ax(2),'Type','line'); assert(abs(h.YData(end)-1)<1e-12);
    save_experiment_figure(f,folder,'matlab_偏差.png');
    [f,ax]=plot_frame_span(t,ones(size(t))*.001,.002,cfg); figs(end+1)=f;
    h=findobj(ax,'Type','line'); assert(all(h.YData==1));
    save_experiment_figure(f,folder,'matlab_跨度.png');
    assert(isempty(mark_motion_phases(ax,[NaN,NaN])));
    expect_error(@()mark_motion_phases(ax,[1,0]));
    expect_error(@()plot_joint_angles(t,q(1:end-1,:),cfg));
    expect_error(@()plot_position_comparison(t,{p},{'a','b'},cfg));
    expect_error(@()plot_pose_errors(t,-t,t,cfg));
    expect_error(@()save_experiment_figure(f,folder,'../bad.png'));
    expect_error(@()save_experiment_figure(f,folder,'bad.exe'));
    assert(all(isgraphics(figs))); % 保存不会隐式关闭图窗
catch exception
    delete(figs(isgraphics(figs))); rethrow(exception);
end
delete(figs);
% 测试图留在临时目录，供人工检查；不覆盖仓库中的实验图片。
fprintf('All plotting tests passed. Test figures: %s\n',folder);

function expect_error(action)
failed=false; try, action(); catch, failed=true; end
assert(failed,'无效输入应报错');
end
