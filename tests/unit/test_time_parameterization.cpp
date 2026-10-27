#include "piper_control/trajectory/time_parameterization.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
using namespace piper_control;
void check(bool ok) { if(!ok) throw std::runtime_error("Time parameterization test failed"); }
template<class F> void rejects(F f) {
    bool failed=false; try { f(); } catch(const std::invalid_argument&) { failed=true; }
    check(failed);
}
int main() {
    // 平滑启动会产生极小旋转；不能在exp/log中把它截断成零。
    for(double angle:{1e-12,1e-10,1e-8,1e-6,1e-4}) {
        const Vec3 phi=angle*Vec3(1,-2,3).normalized();
        check((so3_log(so3_exp(phi))-phi).norm()<1e-14);
        Twist xi; xi<<phi,0.2,-0.1,0.3;
        check((se3_log(se3_exp(xi))-xi).norm()<1e-12);
    }
    const auto a=quintic_time_scaling(0,10), b=quintic_time_scaling(10,10);
    check(a.s==0 && b.s==1 && a.ds_dt==0 && b.ds_dt==0 && a.d2s_dt2==0 && b.d2s_dt2==0);
    double prev=-1;
    for(int i=0;i<=100;++i) {
        const auto x=quintic_time_scaling(i/10.0,10);
        const auto y=quintic_time_scaling(i/5.0,20);
        check(x.s>prev && x.ds_dt>=0); prev=x.s;
        check(std::abs(x.s-y.s)<1e-14 && std::abs(x.ds_dt/2-y.ds_dt)<1e-14
              && std::abs(x.d2s_dt2/4-y.d2s_dt2)<1e-14);
    }
    MatXd q(20,2);
    for(int i=0;i<20;++i) { double t=i*.1; q(i,0)=2*t+1; q(i,1)=t*t; }
    const auto d=differentiate_joint_samples(q,.1);
    for(int i=0;i<20;++i) {
        check(std::abs(d.velocity(i,0)-2)<1e-11 && std::abs(d.velocity(i,1)-2*i*.1)<1e-11);
        check(std::abs(d.acceleration(i,0))<1e-10 && std::abs(d.acceleration(i,1)-2)<1e-10);
    }
    rejects([] { quintic_time_scaling(1,0); });
    rejects([] { quintic_time_scaling(-1,10); });
    rejects([] { quintic_time_scaling(11,10); });
    rejects([] { quintic_time_scaling(0,std::numeric_limits<double>::infinity()); });
    rejects([&] { differentiate_joint_samples(q,0); });
    rejects([] { differentiate_joint_samples(MatXd::Zero(3,2),.1); });
    q(1,0)=std::numeric_limits<double>::quiet_NaN();
    rejects([&] { differentiate_joint_samples(q,.1); });
}
