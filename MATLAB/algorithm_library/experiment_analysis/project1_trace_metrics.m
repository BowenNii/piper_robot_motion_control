function m=project1_trace_metrics(t,error,limit)
%PROJECT1_TRACE_METRICS 标量非负误差的时间加权RMS、超限时间占比与最长连续超限。
% 样本i代表[t_i,t_i+1)；末样本不外推。limit=[]表示未设验收容差。
validateattributes(t,{'numeric'},{'column','finite','increasing'});
validateattributes(error,{'numeric'},{'column','finite','nonnegative','numel',numel(t)});
assert(numel(t)>=2);
dt=diff(t);duration=sum(dt);v=error(1:end-1);
m=struct('peak',max(error),'rms',sqrt(sum(v.^2.*dt)/duration), ...
    'over_fraction',NaN,'longest_over_s',NaN);
if isempty(limit),return;end
validateattributes(limit,{'numeric'},{'scalar','finite','positive'});
over=v>limit;m.over_fraction=sum(dt(over))/duration;
current=0;longest=0;
for k=1:numel(dt)
    if over(k),current=current+dt(k);longest=max(longest,current);else,current=0;end
end
m.longest_over_s=longest;
end
