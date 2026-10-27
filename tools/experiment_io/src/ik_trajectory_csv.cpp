#include "experiment_io/ik_trajectory_csv.hpp"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace experiment_io {
void write_ik_trajectory_csv(const std::filesystem::path& path,
    const std::vector<IkTrajectorySample>& samples,
    const piper_control::PiperModel& model, const piper_control::IKOptions& o,
    const piper_control::VecXd& start, const piper_control::VecXd& goal,
    double max_step, const IkTrajectoryTiming& timing)
{
    if (samples.size()<2 || start.size()!=6 || goal.size()!=6
        || model.q_min.size()!=6 || model.q_max.size()!=6)
        throw std::invalid_argument("IK trajectory CSV requires complete six-axis data");
    if (!start.allFinite() || !goal.allFinite() || !model.q_min.allFinite()
        || !model.q_max.allFinite() || !std::isfinite(max_step) || max_step<=0)
        throw std::invalid_argument("Invalid trajectory CSV configuration");
    if (!std::isfinite(timing.duration_s) || timing.duration_s<=0
        || !std::isfinite(timing.velocity_limit_rad_s) || timing.velocity_limit_rad_s<=0
        || !std::isfinite(timing.acceleration_limit_rad_s2) || timing.acceleration_limit_rad_s2<=0)
        throw std::invalid_argument("Invalid trajectory timing configuration");
    std::ostringstream text;
    text.imbue(std::locale::classic());
    text << std::setprecision(17);
    text << "index,s,iterations,position_error_m,rotation_error_rad"
         << ",weighted_sigma_min,max_abs_delta_q_rad";
    for (int i=1;i<=6;++i) text << ",q" << i << "_rad";
    for (int i=1;i<=6;++i) text << ",delta_q" << i << "_rad";
    // 每个位姿按XYZ和逐行旋转矩阵顺序展开。
    for (const char* prefix:{"target","poe"}) {
        for (const char* axis:{"x","y","z"}) text << ',' << prefix << '_' << axis << "_m";
        for (int row=1;row<=3;++row)
            for (int col=1;col<=3;++col) text << ',' << prefix << "_r" << row << col;
    }
    text << ",position_tolerance_m,orientation_tolerance_rad,max_sample_step_rad"
         << ",rotation_weight,dls_lambda,ik_max_joint_step_rad,max_iterations,sample_count";
    for (const char* prefix:{"q_min","q_max","q_start","q_goal"})
        for (int i=1;i<=6;++i) text << ',' << prefix << i << "_rad";
    text << ",t_s,ds_dt,d2s_dt2";
    for (int i=1;i<=6;++i) text << ",dq" << i << "_rad_s";
    for (int i=1;i<=6;++i) text << ",ddq" << i << "_rad_s2";
    text << ",duration_s,sample_period_s,offline_velocity_limit_rad_s,offline_acceleration_limit_rad_s2";
    if(timing.path_mode!=0 && timing.path_mode!=1) throw std::invalid_argument("Invalid path mode");
    if(timing.path_mode==1) text << ",path_mode";
    text << '\n';
    for (std::size_t k=0;k<samples.size();++k) {
        const auto& r=samples[k];
        if (r.index!=static_cast<int>(k) || !std::isfinite(r.s) || r.s<0 || r.s>1
            || (k>0 && r.s<=samples[k-1].s) || r.iterations<0
            || !std::isfinite(r.error.position_m) || r.error.position_m<0
            || !std::isfinite(r.error.rotation_rad) || r.error.rotation_rad<0
            || !std::isfinite(r.weighted_sigma_min) || r.weighted_sigma_min<0
            || !std::isfinite(r.max_abs_delta_q_rad) || r.max_abs_delta_q_rad<0
            || !r.q_rad.allFinite() || !r.delta_q_rad.allFinite()
            || !piper_control::check_se3(r.target).ok || !piper_control::check_se3(r.fk).ok)
            throw std::invalid_argument("Invalid trajectory CSV sample");
        if (!std::isfinite(r.t_s) || r.t_s<0 || r.t_s>timing.duration_s
            || (k>0 && r.t_s<=samples[k-1].t_s) || !std::isfinite(r.ds_dt)
            || !std::isfinite(r.d2s_dt2) || !r.dq_rad_s.allFinite() || !r.ddq_rad_s2.allFinite()
            || r.dq_rad_s.cwiseAbs().maxCoeff()>timing.velocity_limit_rad_s
            || r.ddq_rad_s2.cwiseAbs().maxCoeff()>timing.acceleration_limit_rad_s2)
            throw std::invalid_argument("Invalid timed sample or offline derivative limit exceeded");
        text << r.index << ',' << r.s << ',' << r.iterations
             << ',' << r.error.position_m << ',' << r.error.rotation_rad
             << ',' << r.weighted_sigma_min << ',' << r.max_abs_delta_q_rad;
        for (double v:r.q_rad) text << ',' << v;
        for (double v:r.delta_q_rad) text << ',' << v;
        for (const auto* T:{&r.target,&r.fk}) {
            for (int i=0;i<3;++i) text << ',' << (*T)(i,3);
            for (int i=0;i<3;++i) for (int j=0;j<3;++j) text << ',' << (*T)(i,j);
        }
        text << ',' << o.position_tolerance << ',' << o.orientation_tolerance
             << ',' << max_step << ',' << o.rotation_weight << ',' << o.lambda
             << ',' << o.max_joint_step << ',' << o.max_iterations << ',' << samples.size();
        for (const auto* q:{&model.q_min,&model.q_max,&start,&goal})
            for (double v:*q) text << ',' << v;
        text << ',' << r.t_s << ',' << r.ds_dt << ',' << r.d2s_dt2;
        for(double v:r.dq_rad_s) text << ',' << v;
        for(double v:r.ddq_rad_s2) text << ',' << v;
        text << ',' << timing.duration_s << ',' << timing.duration_s/(samples.size()-1)
             << ',' << timing.velocity_limit_rad_s << ',' << timing.acceleration_limit_rad_s2;
        if(timing.path_mode==1) text << ",1";
        text << '\n';
    }
    // O_EXCL在系统层原子拒绝已有文件。先整理完整文本，再创建目标。
    int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0644);
    const auto failure=[&] {
        return std::runtime_error("CSV I/O error: "+path.string()+": "+std::strerror(errno));
    };
    if (fd<0) throw failure();
    try {
        const std::string data=text.str();
        std::size_t done=0;
        while (done<data.size()) {
            const auto n=::write(fd,data.data()+done,data.size()-done);
            if (n<0 && errno==EINTR) continue;
            if (n<=0) { if(n==0) errno=EIO; throw failure(); }
            done+=static_cast<std::size_t>(n);
        }
        if (::fsync(fd)!=0) throw failure();
        const int closing=fd;
        fd=-1;
        if (::close(closing)!=0) throw failure();
    } catch (...) {
        if(fd>=0) ::close(fd);
        throw;
    }
}
}
