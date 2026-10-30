function test_project1_closeout_metrics()
% 合成数据检查：时间加权统计、缺失验收容差、双向折线路径距离。
t=[0;1;3;4];e=[0;2;2;0];m=project1_trace_metrics(t,e,1);
assert(abs(m.rms-sqrt(3))<1e-12);
assert(abs(m.over_fraction-.75)<1e-12&&m.longest_over_s==3);
m=project1_trace_metrics(t,e,[]);assert(isnan(m.over_fraction));
d=project1_path_distance([.5 1 0;2 0 0],[0 0 0;1 0 0]);assert(max(abs(d-[1;1]))<1e-12);
% 单向距离为零也可能只走了前半段，反向检查应发现缺失覆盖。
reference=[0 0 0;1 0 0;2 0 0];partial=[0 0 0;1 0 0];
assert(max(project1_path_distance(partial,reference))==0);
assert(max(project1_path_distance(reference,partial))==1);
disp('Project1 closeout metrics PASS');
end
