function distance=project1_path_distance(points,path)
%PROJECT1_PATH_DISTANCE 点到三维折线的最短距离，不平移/旋转空间、不拟合配准。
% 应双向计算，防止实际只走完部分路径却得到接近零的单向距离。
validateattributes(points,{'numeric'},{'2d','ncols',3,'finite'});
validateattributes(path,{'numeric'},{'2d','ncols',3,'finite'});
assert(size(path,1)>=2);distance=inf(size(points,1),1);
for k=1:size(path,1)-1
    a=path(k,:);d=path(k+1,:)-a;den=sum(d.^2);
    if den<eps,projection=repmat(a,size(points,1),1);
    else,u=max(0,min(1,((points-a)*d')/den));projection=a+u.*d;end
    distance=min(distance,sqrt(sum((points-projection).^2,2)));
end
end
