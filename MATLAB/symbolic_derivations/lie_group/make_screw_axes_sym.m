function S_list = make_screw_axes_sym(w,p,h_in,joint_types)
%MAKE_SCREW_AXES_SYM 批量构造6×n符号旋量[omega;v]。
%   S_list=MAKE_SCREW_AXES_SYM(w,p,h_in,joint_types)
%   输入：w 3×n轴方向；p 3×n，转动时为轴上一点[m]，移动时为移动方向；
%         h_in可省略，螺距[m/rad]标量或n维，默认0；
%         joint_types可省略，或cell/string数组：'revolute'/'prismatic'。
%   输出：转动关节S=[w;-cross(w,p)+h*w]，移动关节S=[0;p]。
%   若轴方向含未确定的符号变量，必须显式传joint_types；轴须单位化。
assert(size(w,1)==3 && isequal(size(w),size(p)), 'w和p必须为3×n');
w=sym(w); p=sym(p);
n=size(w,2);
if nargin<3 || isempty(h_in)
    h_list=sym(zeros(1,n));
elseif isscalar(h_in)
    h_list=repmat(sym(h_in),1,n);
else
    h_list=sym(h_in(:).');
    assert(numel(h_list)==n, 'h_in必须为标量或n维');
end
if nargin<4 || isempty(joint_types)
    joint_types=cell(1,n);
    for i=1:n
        if ~isempty(symvar(w(:,i)))
            error('含符号变量的轴必须显式提供joint_types');
        end
        if isequal(w(:,i),sym(zeros(3,1)))
            joint_types{i}='prismatic';
        else
            joint_types{i}='revolute';
        end
    end
else
    joint_types=cellstr(joint_types);
    assert(numel(joint_types)==n, 'joint_types长度必须等于关节数');
end
S_list=sym(zeros(6,n));
for i=1:n
    wi=w(:,i); pi=p(:,i);
    if strcmpi(joint_types{i},'prismatic')
        assert(isequal(wi,sym(zeros(3,1))), '移动关节w必须为零');
        check_unit_if_known(pi,'移动方向');
        S_list(:,i)=[sym(zeros(3,1));pi];
    elseif strcmpi(joint_types{i},'revolute')
        check_unit_if_known(wi,'旋转轴');
        S_list(:,i)=[wi;-skew_sym(wi)*pi+h_list(i)*wi];
    else
        error('joint_types仅支持revolute或prismatic');
    end
end
end

function check_unit_if_known(v,name)
if isempty(symvar(v))
    length_value=double(sqrt(v.'*v));
    assert(isfinite(length_value) && abs(length_value-1)<1e-10, ...
        '%s必须为单位向量',name);
end
end
