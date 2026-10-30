function test_project1_analysis_config()
% 默认JSON真实消费；正容差有效，未知字段/错误版本/负数必须拒绝。
root=setup_piper_paths;repo=fileparts(root);
source=fullfile(repo,'config','project1_analysis.json');cfg=load_project1_analysis_config(source);
assert(isempty(cfg.joint_tolerance_deg)&&isempty(cfg.position_tolerance_mm));
value=jsondecode(fileread(source));value.position_tolerance_mm=1;
folder=tempname;mkdir(folder);path=fullfile(folder,'analysis.json');
write_json(path,value);checked=load_project1_analysis_config(path);assert(checked.position_tolerance_mm==1);
bad=value;bad.position_tolerance_mm=-1;reject(path,bad);
bad=value;bad.schema_version=2;reject(path,bad);
bad=value;bad.scope='hardware';reject(path,bad);
bad=value;bad.unknown_parameter=1;reject(path,bad);
disp('Project1 analysis config PASS');
end
function write_json(path,value)
f=fopen(path,'w');assert(f~=-1);cleanup=onCleanup(@()fclose(f));fprintf(f,'%s',jsonencode(value));
end
function reject(path,value)
write_json(path,value);caught=false;
try,load_project1_analysis_config(path);catch,caught=true;end
assert(caught,'必须拒绝非法分析配置');
end
