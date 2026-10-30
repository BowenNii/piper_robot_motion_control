// 五组方案离线生成器：不连接ROS/CAN；文件不是现有真机发送器输入。
#include <Eigen/SVD>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include "piper_control/trajectory/comparison_profile.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/validation/kinematics_validation.hpp"
using namespace piper_control;
int main(int argc,char** argv) {
    try {
        if((argc!=2 && argc!=16) || std::string(argv[1])=="--help") {
            std::cout<<"Usage: joint_profile_comparison NEW_OUTPUT_DIRECTORY [--start q1..q6 --goal q1..q6]\nOffline only; no collision or dynamics certification.\n";
            return argc==2?0:1;
        }
        const std::filesystem::path out=argv[1];
        if(std::filesystem::exists(std::filesystem::symlink_status(out)) ||
           !std::filesystem::is_directory(out.parent_path()))
            throw std::runtime_error("Use a NEW directory with existing parent; no overwrite");
        const auto model=make_piper_model();
        VecXd q0=VecXd::Zero(6),reference(6);
        reference<<.3,.3,-.3,.3,.3,.5;
        if(argc==16) {
            if(std::string(argv[2])!="--start" || std::string(argv[9])!="--goal")
                throw std::invalid_argument("Expected --start six angles --goal six angles");
            for(int j=0;j<6;++j) {
                std::size_t used=0;q0(j)=std::stod(argv[3+j],&used);
                if(used!=std::string(argv[3+j]).size())throw std::invalid_argument("Bad start angle");
                reference(j)=std::stod(argv[10+j],&used);
                if(used!=std::string(argv[10+j]).size())throw std::invalid_argument("Bad goal angle");
            }
        }
        if(!q0.allFinite() || !reference.allFinite() ||
            (q0.array()<model.q_min.array()).any() || (q0.array()>model.q_max.array()).any() ||
            (reference.array()<model.q_min.array()).any() || (reference.array()>model.q_max.array()).any())
            throw std::invalid_argument("Start/goal outside model limits; no clipping");
        const auto target=forward_poe_space(model.S_list,reference,model.M);
        IKOptions options;
        options.q_min=model.q_min;
        options.q_max=model.q_max;
        options.max_iterations=2000;
        options.position_tolerance=1e-9;
        options.orientation_tolerance=1e-9;
        // 参考角只用于指定共同解支；不是声称从零位的局部IK一定能收敛。
        const auto ik=solve_ik_dls(model.S_list,model.M,target,reference,options);
        IkValidationConfig cfg;
        cfg.position_tolerance_m=1e-9;
        cfg.rotation_tolerance_rad=1e-9;
        if(!ik.success || !check_ik_solution(model.S_list,model.M,ik.q,target,
            model.q_min,model.q_max,cfg).ok) throw std::runtime_error("Endpoint IK failed");
        const VecXd delta=ik.q-q0;
        const double distance=delta.cwiseAbs().maxCoeff(),rad=std::acos(-1.)/180,dt=.02;
        // 方案1..3以速度确定时长；此离线上限不是厂家安全参数。
        const auto baseline=make_comparison_profile(2,distance,5*rad,.2,dt);
        if(!std::filesystem::create_directory(out)) 
        throw std::runtime_error("Cannot create output directory");
        std::ofstream summary(out/"summary.csv");
        summary.exceptions(std::ios::badbit|std::ios::failbit);
        summary<<std::setprecision(17)<<"scheme,motion_duration_s,grid_duration_s,accel_time_s,cruise_time_s,peak_velocity_deg_s,peak_acceleration_deg_s2,min_weighted_sigma\n";
        for(int scheme=1;scheme<=5;++scheme) {
            const double speed=(scheme==1?1:scheme==3?8:5)*rad;
            const auto p=make_comparison_profile(scheme,distance,scheme>=4?baseline.peak_v:speed,
                scheme>=4?baseline.peak_a:.2,dt);
            double minimum=std::numeric_limits<double>::infinity();
            for(int back=0;back<2;++back) {
                const VecXd start=back?ik.q:q0,displacement=back?-delta:delta;
                std::ofstream f(out/("scheme"+std::to_string(scheme)+(back?"_return.csv":"_outbound.csv")));
                f.exceptions(std::ios::badbit|std::ios::failbit);
                f<<std::setprecision(17);
                f<<"t_s,s,ds_dt,d2s_dt2,phase,weighted_sigma_min";
                for(const auto prefix:{"q","dq","ddq","jerk"}) for(int j=1;j<=6;++j)
                    f<<','<<prefix<<j<<(std::string(prefix)=="q"?"_rad":std::string(prefix)=="dq"?"_rad_s":std::string(prefix)=="ddq"?"_rad_s2":"_rad_s3");
                for(const auto axis:{"x","y","z"})f<<",poe_"<<axis<<"_m";
                for(int row=1;row<=3;++row)for(int col=1;col<=3;++col)f<<",poe_r"<<row<<col;
                f<<'\n';
                const int n=static_cast<int>(std::llround(p.grid_duration/dt));
                VecXd previous=start;
                for(int k=0;k<=n;++k) {
                    const double t=k*dt;const auto z=p.at(t);
                    const VecXd q=start+displacement*(z.x/distance);
                    const VecXd v=displacement*(z.v/distance),a=displacement*(z.a/distance),jerk=displacement*(z.jerk/distance);
                    if(!q.allFinite() || (q.array()<model.q_min.array()-1e-12).any() ||
                       (q.array()>model.q_max.array()+1e-12).any() ||
                       (v.cwiseAbs().array()>model.dq_max.array()+1e-12).any() ||
                       v.cwiseAbs().maxCoeff()>p.peak_v+1e-10 || a.cwiseAbs().maxCoeff()>p.peak_a+1e-10 ||
                       (q-previous).cwiseAbs().maxCoeff()>.05)
                        throw std::runtime_error("Offline limits failed; partial directory must not be executed");
                    const auto T=forward_poe_space(model.S_list,q,model.M);
                    MatXd J=adjoint_inverse(T)*jacobian_space(model.S_list,q);
                    J.topRows(3)*=.2;const double sigma=Eigen::JacobiSVD<MatXd>(J).singularValues().tail(1)(0);
                    if(!std::isfinite(sigma)) throw std::runtime_error("Nonfinite sigma");
                    minimum=std::min(minimum,sigma);
                    f<<t<<','<<z.x/distance<<','<<z.v/distance<<','<<z.a/distance<<','<<z.phase<<','<<sigma;
                    for(const VecXd* values:{&q,&v,&a,&jerk}) for(int j=0;j<6;++j)f<<','<<(*values)(j);
                    for(int j=0;j<3;++j)f<<','<<T(j,3);
                    for(int row=0;row<3;++row)for(int col=0;col<3;++col)f<<','<<T(row,col);
                    f<<'\n';previous=q;
                }
                f.close();
            }
            summary<<scheme<<','<<p.duration<<','<<p.grid_duration<<','<<p.ta<<','<<p.tc<<','<<p.peak_v/rad<<','<<p.peak_a/rad<<','<<minimum<<'\n';
            std::cout<<"Scheme "<<scheme<<": "<<p.grid_duration<<" s, peak "<<p.peak_v/rad<<" deg/s, "<<p.peak_a/rad<<" deg/s^2\n";
        }
        summary.close();
        std::ofstream note(out/"OFFLINE_ONLY.txt");note.exceptions(std::ios::badbit|std::ios::failbit);
        note<<"COMPLETE OFFLINE DATASET. NOT APPROVED FOR HARDWARE.\n"
            <<"No collision, torque, latency, watchdog or controller-mode validation.\n"
            <<"Return assumes exact planned endpoint; regenerate from measured state before hardware.\n"
            <<"Scheme 5 has acceleration discontinuities: finite listed jerk excludes impulses.\n"
            <<"Scheme 1..3 phase=0 means whole quintic, not a constant-acceleration phase.\n"
            <<"Scheme 4/5 phase 0/1/2/3=accel/cruise/decel/endpoint hold.\n"
            <<"20ms grid; scheme4/5 end hold <20ms avoids stretching matched peaks.\n"
            <<"IK uses reference seed to fix branch; not online Cartesian IK or a zero-seed convergence test.\n";
        note.close();
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
