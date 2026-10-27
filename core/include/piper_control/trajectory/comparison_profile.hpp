#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace piper_control {
// 离线标量位移规律：长度可用rad，速度rad/s，加速度rad/s²。
// scheme 1..3: 整程五次位置；4: 五次速度过渡；5: 梯形速度。
struct ComparisonState { double x, v, a, jerk; int phase; };
struct ComparisonProfile {
    int scheme;
    double distance, duration, grid_duration, ta, tc, peak_v, peak_a;
    ComparisonState at(double t) const {
        if (!std::isfinite(t) || t<0 || t>grid_duration+1e-9)
            throw std::invalid_argument("Invalid profile time");
        if (t>=duration) return {distance,0,0,0,3};
        if (scheme<=3) {
            const double u=t/duration;
            return {distance*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5)),
                distance/duration*30*u*u*(1-u)*(1-u),
                distance/(duration*duration)*60*u*(1-u)*(1-2*u),
                distance/std::pow(duration,3)*(60-360*u+360*u*u),0};
        }
        const auto accel=[&](double z) -> ComparisonState {
            if (scheme==5) return {.5*peak_a*z*z,peak_a*z,peak_a,0,0};
            const double u=z/ta;
            const double h=10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5);
            const double H=2.5*std::pow(u,4)-3*std::pow(u,5)+std::pow(u,6);
            return {peak_v*ta*H,peak_v*h,peak_v/ta*30*u*u*(1-u)*(1-u),
                peak_v/(ta*ta)*60*u*(1-u)*(1-2*u),0};
        };
        if (t<ta) return accel(t);
        if (t<ta+tc) return {.5*peak_v*ta+peak_v*(t-ta),peak_v,0,0,1};
        auto z=accel(duration-t);
        return {distance-z.x,z.v,-z.a,z.jerk,2};
    }
};
inline ComparisonProfile make_comparison_profile(int scheme,double distance,
        double speed,double acceleration,double dt=.02) {
    if (scheme<1 || scheme>5 || !std::isfinite(distance) || distance<=0 ||
        !std::isfinite(speed) || speed<=0 || !std::isfinite(acceleration) || acceleration<=0 ||
        !std::isfinite(dt) || dt<=0) throw std::invalid_argument("Invalid profile inputs");
    ComparisonProfile p{scheme,distance,0,0,0,0,0,0};
    if (scheme<=3) {
        // 先同时满足速度和加速度；时长向上取整，不超过约束。
        p.duration=std::ceil(std::max(1.875*distance/speed,
            std::sqrt((10/std::sqrt(3.0))*distance/acceleration))/dt)*dt;
        p.peak_v=1.875*distance/p.duration;
        p.peak_a=(10/std::sqrt(3.0))*distance/(p.duration*p.duration);
    } else {
        const double factor=scheme==4 ? 1.875 : 1.0;
        p.peak_v=std::min(speed,std::sqrt(distance*acceleration/factor));
        p.peak_a=acceleration;p.ta=factor*p.peak_v/acceleration;
        p.tc=std::max(0.0,distance/p.peak_v-p.ta);
        p.duration=2*p.ta+p.tc;
    }
    // 解析运动完成后保持终点不足一个采样周期，不改变4/5的匹配峰值。
    p.grid_duration=std::ceil(p.duration/dt-1e-10)*dt;
    return p;
}
} // namespace piper_control
