#include "piper_control/trajectory/comparison_profile.hpp"
#include <iostream>
#include <stdexcept>
using namespace piper_control;
void check(bool x) { if(!x) throw std::runtime_error("Profile check failed"); }
int main() {
    for (int scheme=1;scheme<=5;++scheme) for(double d:{.001,.3,.5}) {
        const auto p=make_comparison_profile(scheme,d,.08,.04);
        check(p.peak_v<=.08+1e-12 && p.peak_a<=.04+1e-12);
        check(p.at(0).x==0 && p.at(0).v==0);
        check(p.at(p.grid_duration).x==d && p.at(p.grid_duration).v==0);
        double prev=-1;
        for(int k=0;k<1000;++k) {
            double t=p.duration*k/1000.;auto s=p.at(t);
            check(s.x>=prev-1e-12 && s.v>=-1e-12 && s.v<=p.peak_v+1e-10);
            check(std::abs(s.a)<=p.peak_a+1e-10);prev=s.x;
            if(t>1e-5 && t<p.duration-1e-5 &&
                (scheme<=3 || (std::abs(t-p.ta)>1e-5 && std::abs(t-p.ta-p.tc)>1e-5))) {
                const double h=1e-6;auto l=p.at(t-h),r=p.at(t+h);
                check(std::abs((r.x-l.x)/(2*h)-s.v)<1e-6);
                check(std::abs((r.v-l.v)/(2*h)-s.a)<1e-6);
            }
        }
        if(scheme==4) for(double t:{p.ta,p.ta+p.tc}) {
            auto l=p.at(t-1e-9),r=p.at(t+1e-9);
            check(std::abs(l.x-r.x)<1e-8 && std::abs(l.v-r.v)<1e-8);
            check(std::abs(l.a-r.a)<1e-8 && std::abs(l.jerk-r.jerk)<1e-6);
        }
    }
    for(int s:{0,6}) {bool caught=false;try{make_comparison_profile(s,.5,.1,.1);}catch(const std::invalid_argument&){caught=true;}check(caught);}
    std::cout<<"Five profiles and short-distance cases passed\n";
}
