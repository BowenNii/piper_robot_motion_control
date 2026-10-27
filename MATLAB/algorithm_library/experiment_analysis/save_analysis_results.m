function paths=save_analysis_results(folder,tables,variables,cfg)
%SAVE_ANALYSIS_RESULTS 显式保存统计CSV与MAT，保存前先检查全部目标。
% 输入：folder；tables标量结构（字段名=CSV基础名，值=table）；
% variables标量结构，字段作为MAT变量；cfg可省略。
% cfg.mat_name默认analysis_results.mat；cfg.overwrite默认false。
% 输出：paths结构包含CSV/MAT完整路径；不保存图片、不关闭图窗。
% 用法：save_analysis_results(dir,struct('summary',summary), ...
%       struct('summary',summary,'cfg',cfg),struct('overwrite',true));
% 同名覆盖必须在主脚本显式开启；提前检查不保证操作系统层面的原子写入。
if nargin<4, cfg=struct; end
if ~isfield(cfg,'overwrite'), cfg.overwrite=false; end%默认禁止覆盖已有文件
if ~isfield(cfg,'mat_name'), cfg.mat_name='analysis_results.mat'; end%没写mat_name就默认叫analysis_results.mat
validateattributes(cfg.overwrite,{'logical'},{'scalar'});%校验overwrite必须是单个逻辑值（true/false）
assert(isstruct(tables) && isscalar(tables) && isstruct(variables) && isscalar(variables), ...
    'tables和variables必须为标量结构');
[parent,stem,ext]=fileparts(cfg.mat_name);%文件名拆成目录路径 + 文件名主体 + 后缀三部分
assert(isempty(parent) && ~isempty(stem) && strcmpi(ext,'.mat'), ...
    'mat_name必须是不含目录的MAT文件名');%不能包含目录路径，文件名主体不能为空，后缀必须是 .mat
folder=char(string(folder)); 
assert(~isempty(folder),'输出目录不能为空');
names=fieldnames(tables); %提取tables结构体的所有字段名，就是每个 CSV 文件的基础名称
outputs=cell(numel(names)+1,1);%创建一个元胞数组outputs，长度 = 表格数量 + 1，多出来的 1 个位置用来存 MAT 文件的路径，为后续的文件存在性检查、正式写入做准备。
for k=1:numel(names)%逐个校验表格并生成输出路径列表
    assert(istable(tables.(names{k})),'CSV输出必须是table');
    outputs{k}=fullfile(folder,[names{k},'.csv']);
end
outputs{end}=fullfile(folder,cfg.mat_name);
for k=1:numel(outputs)%统一做文件存在性与防覆盖校验
    assert(~isfolder(outputs{k}) && (cfg.overwrite || ~isfile(outputs{k})), ...
        '输出已存在或是目录：%s',outputs{k});
end
if ~isfolder(folder), [ok,message]=mkdir(folder); assert(ok,'%s',message); end
[ok,info]=fileattrib(folder); assert(ok,'无法读取输出目录');
paths=struct;
for k=1:numel(names)%自动创建输出目录,逐个导出 CSV
    paths.(names{k})=fullfile(info.Name,[names{k},'.csv']);
    writetable(tables.(names{k}),paths.(names{k}));
end
paths.mat=fullfile(info.Name,cfg.mat_name);
save(paths.mat,'-struct','variables');
end
