function report = check_feedback_data(data, required_columns)
%CHECK_FEEDBACK_DATA 检查已读取的数据表是否合法、完整
% 不修改数据、不删除异常行、不主动抛错，最终输出一份结构化的校验报告。
% 输入：data MATLAB table；required_columns 必需数值列名，cell或string。
% 输出：report.ok、messages、missing_columns、invalid_rows、sample_count。
% ok最终是否全部通过校验，布尔值messages所有问题的文字描述列表
% missing_columns缺失的必填列名列表
% invalid_rows存在无效值（NaN/Inf）的行号，sample_count数据总行数
% 用法：r=check_feedback_data(data,{'t_s','q1_rad'});
%       if ~r.ok, error('%s',strjoin(r.messages,'；')); end
% 格式错误由主脚本决定是否停止；本函数只报告，不抛出数据质量警告。
report=struct('ok',false,'messages',{{}},'missing_columns',{{}}, ...
    'invalid_rows',[],'sample_count',0);
if ~istable(data)
    report.messages={'输入必须是table'}; return;
end
names=cellstr(string(required_columns));
report.sample_count=height(data);
report.missing_columns=setdiff(names,data.Properties.VariableNames,'stable');
if height(data)==0, report.messages{end+1}='数据表为空'; end
if ~isempty(report.missing_columns)
    report.messages{end+1}=['缺少列：',strjoin(report.missing_columns,', ')];
end
bad=false(height(data),1);
for k=1:numel(names)
    if ~ismember(names{k},data.Properties.VariableNames), continue; end
    x=data.(names{k});
    if ~isnumeric(x) || ~isreal(x) || size(x,2)~=1
        report.messages{end+1}=['列必须是实数单列：',names{k}];
        continue;
    end
    bad=bad | ~isfinite(x);
end
report.invalid_rows=find(bad);
if any(bad), report.messages{end+1}='必需列中存在NaN或Inf'; end
report.ok=isempty(report.messages);
end
