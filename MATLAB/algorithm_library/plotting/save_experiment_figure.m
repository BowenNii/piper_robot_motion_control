function path=save_experiment_figure(fig,folder,name,cfg)
%SAVE_EXPERIMENT_FIGURE 显式保存图窗，不关闭图窗、不修改其格式。
% 输入：fig；folder目标目录；name文件名（无扩展名时默认.png）。
% cfg.resolution默认150dpi；cfg.overwrite默认false，防止覆盖已有结果。
% 支持png/jpg/jpeg/pdf/fig；目录不存在会创建；name不得包含路径。
% 输出：保存文件的完整路径。FIG可再次用openfig编辑，PDF为矢量导出。
if nargin<4, cfg=struct; end
assert(isscalar(fig) && isgraphics(fig,'figure'),'输入必须是有效图窗');
if ~isfield(cfg,'resolution'), cfg.resolution=150; end
if ~isfield(cfg,'overwrite'), cfg.overwrite=false; end
validateattributes(cfg.resolution,{'numeric'},{'scalar','finite','positive'});
validateattributes(cfg.overwrite,{'logical'},{'scalar'});
folder=char(string(folder)); name=char(string(name));
assert(~isempty(folder) && ~isempty(name),'目录与文件名不能为空');
[parent,stem,ext]=fileparts(name);
assert(isempty(parent) && ~isempty(stem) && ~any(name=='/') ...
    && ~any(name==char(92)),'name只能是文件名，不得包含目录');
if isempty(ext), ext='.png'; name=[name,ext]; end
assert(ismember(lower(ext),{'.png','.jpg','.jpeg','.pdf','.fig'}),'不支持的图片格式');
if ~isfolder(folder)
    [ok,message]=mkdir(folder); assert(ok,'%s',message);
end
% 转为绝对目录，方便换工作目录后继续引用结果。
[ok,info]=fileattrib(folder); assert(ok,'无法读取目标目录');
path=fullfile(info.Name,name);
assert(~isfolder(path),'输出路径是已有目录');
assert(cfg.overwrite || ~isfile(path),'文件已存在；换名或显式设置overwrite=true');
drawnow;
if strcmpi(ext,'.fig')
    savefig(fig,path);
elseif strcmpi(ext,'.pdf')
    exportgraphics(fig,path,'ContentType','vector');
else
    exportgraphics(fig,path,'Resolution',cfg.resolution);
end
end
