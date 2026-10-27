function cfg=plot_config(cfg)
% 内部默认配置；实验主脚本可以覆盖这些字段。
defaults=struct('visible','on','font_name','Noto Sans CJK SC', ...
    'font_size',11,'line_width',1.2,'position',[60,60,1120,780], ...
    'title','','angle_unit','deg','length_unit','mm', ...
    'legend_location','northeast','bounds_s',[]);
names=fieldnames(defaults);
for k=1:numel(names)
    if ~isfield(cfg,names{k}), cfg.(names{k})=defaults.(names{k}); end
end
assert(ismember(string(cfg.angle_unit),["rad","deg"]),'角度显示单位必须rad或deg');
assert(ismember(string(cfg.length_unit),["m","mm"]),'长度显示单位必须m或mm');
validateattributes(cfg.font_size,{'numeric'},{'scalar','finite','positive'});
validateattributes(cfg.line_width,{'numeric'},{'scalar','finite','positive'});
end
