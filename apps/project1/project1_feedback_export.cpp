// 完整项目1日志解码入口：只读candump -L，不打开CAN或ROS。
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include "hal/can_log_parser.hpp"
#include "experiment_io/fk_csv_writer.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/validation/pose_error.hpp"
int signed16(const std::array<std::uint8_t,8>& d,int i) {
    const int u=int(d[i])*256+d[i+1];return u>=32768?u-65536:u;
}
int main(int argc,char** argv) {
    using namespace piper_control;
    try {
        if(argc!=3) {std::cerr<<"Usage: project1_feedback_export INPUT.log NEW_DIRECTORY\n";return 1;}
        const std::filesystem::path out=argv[2];
        std::ifstream input(argv[1]);if(!input)throw std::runtime_error("Input unavailable");
        if(std::filesystem::exists(std::filesystem::symlink_status(out)) ||
            !std::filesystem::is_directory(out.parent_path()) || !std::filesystem::create_directory(out))
            throw std::runtime_error("Output must be a new directory with existing parent");
        experiment_io::FkCsvWriter fk(out/"feedback_fk.csv");
        std::ofstream motors(out/"motors.csv"),status(out/"can_status.csv"),commands(out/"can_commands.csv");
        for(auto* f:{&motors,&status,&commands}) {f->exceptions(std::ios::failbit|std::ios::badbit);*f<<std::setprecision(17);}
        motors<<"timestamp_s,joint,motor_velocity_reported,motor_position_reported,current_A,torque_estimated_Nm\n";
        status<<"timestamp_s,ctrl_mode,arm_status,mode_feedback,teach_status,motion_status,trajectory_num,err_status\n";
        commands<<"timestamp_s,joint,q_rad\n";
        // 2ms是报告阈值，不先删除超2ms样本。50ms只是离线组装上限。
        hal::PiperFeedbackAssembler assembler(.05, true);
        const auto model=make_piper_model();
        std::size_t lines=0,bad=0,count=0,motor_count=0,over2=0,cmd_count=0;
        double first=0,last=-1,spanmax=0;std::string line;
        while(std::getline(input,line)) {
            ++lines;hal::TimedCanFrame frame;
            if(!hal::parse_can_log_line(line,frame)) {++bad;assembler.reset();continue;}
            if(frame.interface_name!="can0")continue;
            if(frame.timestamp_seconds<last)throw std::runtime_error("CAN timestamps reversed");
            last=frame.timestamp_seconds;
            if(frame.frame.length!=8)continue;
            const auto id=frame.frame.id;const auto& d=frame.frame.data;
            if(id>=0x251 && id<=0x256) {
                const int j=int(id-0x250);const double current=signed16(d,2)*.001;
                motors<<last<<','<<j<<','<<signed16(d,0)*.001<<','<<hal::decode_int32_be(d,4)*.001
                    <<','<<current<<','<<current*(j<=3?1.18125/4:.95844)<<'\n';++motor_count;
            }
            if(id==0x2A1) {
                const auto s=hal::decode_status(frame.frame);
                status<<last<<','<<int(s.control_mode)<<','<<int(s.arm_status)<<','<<int(s.motion_mode)
                    <<','<<int(s.teach_status)<<','<<int(s.motion_status)<<','<<int(s.trajectory_point)<<','<<s.fault_code<<'\n';
            }
            if(id>=0x155 && id<=0x157) {
                for(int k=0;k<2;++k)commands<<last<<','<<2*(id-0x155)+k+1<<','
                    <<hal::decode_int32_be(d,k*4)*std::acos(-1.)/180000.<<'\n';
                ++cmd_count;
            }
            const auto s=assembler.push(frame);if(!s)continue;
            if(count++==0)first=s->timestamp_seconds;
            experiment_io::FkValidationResult r;
            r.t_s=s->timestamp_seconds-first;r.timestamp_s=s->timestamp_seconds;r.frame_span_s=s->span_seconds;
            for(int j=0;j<6;++j)r.q_rad(j)=s->q_rad[j];
            r.T_poe=forward_poe_space(model.S_list,r.q_rad,model.M);
            const auto& p=s->end_pose.position_m;const auto& e=s->end_pose.rpy_rad;
            r.feedback_rpy_rad=e;
            r.T_feedback=se3_from_rt(rot_z(e[2])*rot_y(e[1])*rot_x(e[0]),Vec3(p[0],p[1],p[2]));
            r.error=pose_error(r.T_poe,r.T_feedback);fk.write(r);
            spanmax=std::max(spanmax,s->span_seconds);if(s->span_seconds>.002)++over2;
        }
        if(input.bad() || !count)throw std::runtime_error("No complete FK samples or input I/O failure");
        fk.close();motors.close();status.close();commands.close();
        const auto& d=assembler.diagnostics();
        std::ofstream report(out/"decode_summary.json");report.exceptions(std::ios::badbit|std::ios::failbit);
        report<<std::setprecision(17)<<"{\"complete\":true,\"lines\":"<<lines<<",\"invalid_lines\":"<<bad
            <<",\"fk_samples\":"<<count<<",\"motor_frames\":"<<motor_count<<",\"command_frames\":"<<cmd_count
            <<",\"span_over_2ms\":"<<over2<<",\"max_span_s\":"<<spanmax
            <<",\"discarded_groups\":"<<d.discarded_groups<<",\"discarded_frames\":"<<d.discarded_frames
            <<",\"window_resets_50ms\":"<<d.over_window<<",\"duplicate_resets\":"<<d.duplicate_ids
            <<",\"ordered_cycles\":true,\"cycle_resets\":"<<d.cycle_resets
            <<",\"unaligned_frames\":"<<d.unaligned_frames
            <<",\"pending_frames\":"<<assembler.pending_frame_count()<<"}\n";report.close();
        std::cout<<"Decoded "<<count<<" FK samples, "<<motor_count<<" motor frames; >2ms groups "<<over2<<'\n';
    }catch(const std::exception& e){std::cerr<<e.what()<<"; partial outputs must not be used\n";return 1;}
}
