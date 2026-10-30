function f=view_project1_figure(number)
%VIEW_PROJECT1_FIGURE 打开收尾报告第1～10张可交互图，点击曲线读取数据。
% 例：view_project1_figure(2)；关闭数据提示后可使用放大镜或zoom工具。
validateattributes(number,{'numeric'},{'scalar','integer','>=',1,'<=',10});
here=fileparts(mfilename('fullpath'));repo=fileparts(fileparts(fileparts(fileparts(here))));
folder=fullfile(repo,'experiments','project1','2026-10-27_movej_repeat','project1_closeout','matlab','figures');
files=dir(fullfile(folder,sprintf('%02d_*.fig',number)));assert(numel(files)==1,'找不到唯一图文件');
f=openfig(fullfile(folder,files.name),'new','visible');
f.WindowState='maximized';datacursormode(f,'on');
end
