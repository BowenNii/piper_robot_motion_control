function cfg=load_project1_analysis_config(path)
%LOAD_PROJECT1_ANALYSIS_CONFIG 仅离线验收容差，null表示未指定，不改硬件配置。
value=jsondecode(fileread(path));
assert(isstruct(value)&&isscalar(value),'配置必须是JSON对象');
keys={'joint_tolerance_deg','position_tolerance_mm','orientation_tolerance_deg'};
allowed=[{'schema_version','scope','description'},keys];
assert(all(ismember(fieldnames(value),allowed)),'配置包含未知字段');
assert(isfield(value,'schema_version')&&isequal(value.schema_version,1),'配置版本必须为1');
assert(isfield(value,'scope')&&strcmp(value.scope,'offline_analysis_only'),'只允许离线分析配置');
cfg=struct;
for k=1:numel(keys)
    name=keys{k};assert(isfield(value,name),'缺少字段：%s',name);
    v=value.(name);
    assert(isnumeric(v),'容差必须是数字或null：%s',name);
    if ~isempty(v),validateattributes(v,{'numeric'},{'scalar','real','finite','positive'});end
    cfg.(name)=v;
end
end
