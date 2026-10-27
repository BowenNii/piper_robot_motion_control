function report = check_se3(T, tolerance)
%CHECK_SE3 检查单个4×4位姿矩阵是否属于SE(3)。
% 输入：T旋转无单位，平移使用m；tolerance默认1e-8。
% 输出：ok、orthogonality_error、determinant、last_row_error、message。
% 数值容差用于检查矩阵结构，不是位置精度容差；不自动修复矩阵。
if nargin<2, tolerance=1e-8; end
validateattributes(tolerance,{'numeric'},{'real','finite','scalar','positive'});
report=struct('ok',false,'orthogonality_error',NaN,'determinant',NaN, ...
    'last_row_error',NaN,'message','矩阵必须是有限实数4×4');
if ~isnumeric(T) || ~isreal(T) || ~isequal(size(T),[4,4]) ...
        || ~all(isfinite(T(:))), return; end
R=T(1:3,1:3);
report.orthogonality_error=norm(R'*R-eye(3),'fro');
report.determinant=det(R);
report.last_row_error=norm(T(4,:)-[0,0,0,1],Inf);
report.ok=report.orthogonality_error<=tolerance ...
    && abs(report.determinant-1)<=tolerance ...
    && report.last_row_error<=tolerance;
report.message='旋转正交性、行列式或末行不合格';
if report.ok, report.message='通过'; end
end
