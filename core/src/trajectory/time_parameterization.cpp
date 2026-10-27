#include "piper_control/trajectory/time_parameterization.hpp"
#include <cmath>
#include <stdexcept>

namespace piper_control {
//五次多项式时间缩放,输入时间 `t` 和总时长 `duration`，
//输出包含路径进度 `s`、进度速度 `v`、进度加速度 `a`的状态
//实现起点、终点速度、加速度都为 0 的平滑加减速。
PathTimeState quintic_time_scaling(double t, double duration) {
    if (!std::isfinite(t) || !std::isfinite(duration) || duration<=0 || t<0 || t>duration)
        throw std::invalid_argument("Invalid quintic time/duration");
    const double u=t/duration;//把实际时间归一化到 `[0,1]` 区间；
//经典五次多项式位置函数`s(x) = 10x³ - 15x⁴ + 6x⁵`
//是轨迹规划最常用的平滑曲线，保证位置、速度、加速度在起点终点都连续且为 0，启停无冲击；
    const auto progress=[](double x) { return x*x*x*(10+x*(-15+6*x)); };
//对称计算优化：前半段直接算，后半段用 `1 - progress(1-u)` 对称计算。
    const double s=u<=0.5 ? progress(u) : 1-progress(1-u);
    const double v=30*u*u*(1-u)*(1-u)/duration;
    const double a=60*u*(1-u)*(1-2*u)/duration/duration;
    if (!std::isfinite(v) || !std::isfinite(a))
        throw std::invalid_argument("Quintic derivative overflow");
    return {s,v,a};
}
//关节序列数值微分
//对均匀时间采样的关节角度序列做数值微分，逐点输出关节速度和加速度，离散轨迹后处理的通用工具
JointTimeDerivatives differentiate_joint_samples(const MatXd& q, double dt) {
    if (q.rows()<4 || q.cols()<1 || !q.allFinite() || !std::isfinite(dt) || dt<=0)
        throw std::invalid_argument("Invalid uniformly timed joint samples");
    const Eigen::Index n=q.rows();
    JointTimeDerivatives out{MatXd(n,q.cols()),MatXd(n,q.cols())};
    for (Eigen::Index i=1;i<n-1;++i) {
        out.velocity.row(i)=(q.row(i+1)-q.row(i-1))/(2*dt);
        out.acceleration.row(i)=(q.row(i+1)-2*q.row(i)+q.row(i-1))/dt/dt;
    }
    out.velocity.row(0)=(-11*q.row(0)+18*q.row(1)-9*q.row(2)+2*q.row(3))/(6*dt);
    out.velocity.row(n-1)=(11*q.row(n-1)-18*q.row(n-2)+9*q.row(n-3)-2*q.row(n-4))/(6*dt);
    out.acceleration.row(0)=(2*q.row(0)-5*q.row(1)+4*q.row(2)-q.row(3))/dt/dt;
    out.acceleration.row(n-1)=(2*q.row(n-1)-5*q.row(n-2)+4*q.row(n-3)-q.row(n-4))/dt/dt;
    if (!out.velocity.allFinite() || !out.acceleration.allFinite())
        throw std::invalid_argument("Joint derivative overflow");
    return out;
}
} // namespace piper_control
