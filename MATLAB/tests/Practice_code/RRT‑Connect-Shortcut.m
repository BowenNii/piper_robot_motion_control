clear; clc; close all;
%% 场景参数
q_start = [0,0];
q_goal  = [9,9];
step    = 1.0;
maxIter = 3000;
thresh  = 1.0;
% 障碍物 [xmin ymin xmax ymax]
obs = [4,2,6,7];

%% 工具函数
function idx = nearest(tree, qr)
    d = vecnorm(tree - qr,2,2);
    [~,idx] = min(d);
end

function qnew = steer(qn, qr, step)
    dir = qr - qn;
    if(norm(dir)<1e-12)
        qnew = qn;
        return;
    end
    dir = dir / norm(dir);
    qnew = qn + step*dir;
end

function ok = collision_free(q1,q2,obs)
    ok = true;
    for t = linspace(0,1,20)
        p = q1*(1-t)+q2*t;
        if p(1)>=obs(1)&&p(1)<=obs(3)&&p(2)>=obs(2)&&p(2)<=obs(4)
            ok=false;
            return;
        end
    end
end

%% RRT-Connect 主函数
function path = rrt_connect(q_start,q_goal,step,maxIter,thresh,obs)
    treeA = q_start;
    parentA = 0;   %根节点父=0
    treeB = q_goal;
    parentB = 0;
    found = false;
    path = [];

    for iter = 1:maxIter
        q_rand = [9*rand(),9*rand()];
        idxA = nearest(treeA,q_rand);
        q_nearA = treeA(idxA,:);
        q_newA = steer(q_nearA,q_rand,step);
        if collision_free(q_nearA,q_newA,obs)
            treeA(end+1,:) = q_newA;
            parentA(end+1) = idxA;
            idxB = nearest(treeB,q_newA);
            q_nearB = treeB(idxB,:);
            q_newB = q_nearB;
            while true
                q_tmp = steer(q_newB,q_newA,step);
                if ~collision_free(q_newB,q_tmp,obs)
                    break;
                end
                treeB(end+1,:) = q_tmp;
                parentB(end+1) = size(treeB,1)-1;
                q_newB = q_tmp;
                if norm(q_newB - q_newA) < thresh
                    found = true;
                    break;
                end
            end
        end
        if found
            % 回溯A：先判断，再读取节点
            pa = size(treeA,1);
            pA = [];
            while true
                if pa == 0, break; end   %【修复】先判断0，再访问数组
                pA(end+1,:)=treeA(pa,:);
                pa = parentA(pa);
            end
            pA = flipud(pA);

            % 回溯B
            pb = size(treeB,1);
            pB = [];
            while true
                if pb ==0, break; end
                pB(end+1,:)=treeB(pb,:);
                pb = parentB(pb);
            end
            path = [pA;pB(2:end,:)];
            break;
        end
        %交换两棵树
        tempT = treeA; tempP = parentA;
        treeA = treeB; parentA = parentB;
        treeB = tempT; parentB = tempP;
    end
end

%% Shortcut剪枝
function path_out = shortcut_smooth(path_in, obs, iter_num)
    path_out = path_in;
    N = size(path_out,1);
    for k = 1:iter_num
        if N <=2, break; end
        i = randi(N-1);
        j = randi([i+1,N]);
        pi = path_out(i,:);
        pj = path_out(j,:);
        if collision_free(pi,pj,obs)
            path_out = [path_out(1:i,:); path_out(j:end,:)];
            N = size(path_out,1);
        end
    end
end

%% ==========主程序==========
path_raw = rrt_connect(q_start,q_goal,step,maxIter,thresh,obs);
if isempty(path_raw)
    disp('未找到可行路径，请增大maxIter');
    return;
end
shortcut_iter = 200;
path_smooth = shortcut_smooth(path_raw, obs, shortcut_iter);

%% ================= B样条平滑 + 碰撞复检 =================
function [path_bspline, ok_smooth] = bspline_smooth_path(path_ctrl, obs, sample_num)
% path_ctrl: N×2 控制点(shortcut输出路径)
% sample_num: B样条上采样点数
% ok_smooth: true=平滑后全程无碰撞；false=平滑后碰撞，不可用
Np = size(path_ctrl,1);
path_bspline = [];
ok_smooth = false;
if Np <3
    path_bspline = path_ctrl;
    ok_smooth = true;
    return;
end

% 三次B样条
k = 3;
n = Np -1;
m = n + k +1;
% 均匀节点向量
U = linspace(0,1,m);

t_list = linspace(U(k+1), U(end-k), sample_num);
path_bspline = zeros(sample_num,2);

for ti = 1:sample_num
    t = t_list(ti);
    % Cox-de-Boor算法
    p = zeros(1,2);
    for i = 0:n
        Ni = BsplineBasis(i,k,t,U);
        p = p + Ni * path_ctrl(i+1,:);
    end
    path_bspline(ti,:) = p;
end

% 碰撞复检：逐段检查B样条曲线采样点之间线段
ok_smooth = true;
for i =1:sample_num-1
    p1 = path_bspline(i,:);
    p2 = path_bspline(i+1,:);
    if ~collision_free(p1,p2,obs)
        ok_smooth = false;
        break;
    end
end
end

function val = BsplineBasis(i,k,t,U)
% Cox-de-Boor 基函数递归
if k ==1
    if (U(i+1) <= t) && (t < U(i+2))
        val =1;
    else
        val =0;
    end
else
    d1 = U(i+k) - U(i+1);
    d2 = U(i+k+1) - U(i+2);
    term1 =0;
    term2 =0;
    if abs(d1) > 1e-12
        term1 = (t-U(i+1))/d1 * BsplineBasis(i, k-1, t, U);
    end
    if abs(d2) >1e-12
        term2 = (U(i+k+1)-t)/d2 * BsplineBasis(i+1, k-1, t, U);
    end
    val = term1 + term2;
end
end

%% --------执行B样条---------
sample_bs = 60;
[path_bs, smooth_ok] = bspline_smooth_path(path_smooth, obs, sample_bs);

figure('Position',[200,200,600,500]);
hold on; axis equal;
rectangle('Position',[obs(1),obs(2),obs(3)-obs(1),obs(4)-obs(2)],'FaceColor',[0.2 0.2 0.2]);
plot(q_start(1),q_start(2),'go','MarkerSize',10);
plot(q_goal(1),q_goal(2),'ro','MarkerSize',10);
plot(path_smooth(:,1),path_smooth(:,2),'m-','LineWidth',1.2,'DisplayName','Shortcut折线');
if smooth_ok
    plot(path_bs(:,1),path_bs(:,2),'c-','LineWidth',1.8,'DisplayName','B样条平滑');
    title('Shortcut + B样条平滑(无碰撞)');
else
    disp('B样条平滑后发生碰撞，放弃平滑，使用shortcut路径');
    title('B样条碰撞，仅展示shortcut路径');
end
legend;
grid on;

% 最终选用路径：平滑成功就用B样条，失败回退shortcut
if smooth_ok
    path_final = path_bs;
else
    path_final = path_smooth;
end

fprintf('最终路径点数：%d\n',size(path_final,1));

%%绘图
figure('Position',[100,100,900,420]);
subplot(1,2,1); hold on; axis equal;
rectangle('Position',[obs(1),obs(2),obs(3)-obs(1),obs(4)-obs(2)],'FaceColor',[0.2 0.2 0.2]);
plot(q_start(1),q_start(2),'go','MarkerSize',10);
plot(q_goal(1),q_goal(2),'ro','MarkerSize',10);
plot(path_raw(:,1),path_raw(:,2),'b-','LineWidth',1.5);
title('RRT-Connect原始路径(曲折)');

subplot(1,2,2); hold on; axis equal;
rectangle('Position',[obs(1),obs(2),obs(3)-obs(1),obs(4)-obs(2)],'FaceColor',[0.2 0.2 0.2]);
plot(q_start(1),q_start(2),'go','MarkerSize',10);
plot(q_goal(1),q_goal(2),'ro','MarkerSize',10);
plot(path_smooth(:,1),path_smooth(:,2),'m-','LineWidth',1.5);
title('Shortcut剪枝后路径');

fprintf('原始路径点数：%d \n',size(path_raw,1));
fprintf('剪枝后路径点数：%d \n',size(path_smooth,1));
