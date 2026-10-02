#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "hal/piper_can_decoder.hpp"

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/robot_model/piper_model.hpp"

int main()
{
    using namespace piper_control;

    const PiperModel model = make_piper_model();

    // 2026-10-01 的真实静态报文：离线解码，不连接或控制机械臂。
    const std::array<hal::CanFrame, 3> joint_frames{{
        {0x2A5, {0, 0, 0, 0, 0xFF, 0xFF, 0xF8, 0x31}},
        {0x2A6, {0, 0, 0x07, 0xB3, 0xFF, 0xFF, 0xFA, 0x7C}},
        {0x2A7, {0, 0, 0x56, 0x5E, 0xFF, 0xFF, 0xF5, 0xD0}}
    }};
    const std::array<hal::CanFrame, 3> pose_frames{{
        {0x2A2, {0, 0, 0xC8, 0x3E, 0xFF, 0xFF, 0xFC, 0xB4}},
        {0x2A3, {0, 0x02, 0x93, 0x9C, 0xFF, 0xFD, 0x73, 0xD3}},
        {0x2A4, {0, 0x01, 0x1B, 0x0A, 0xFF, 0xFD, 0x74, 0x0B}}
    }};
    const auto q_feedback = hal::decode_joint_angles(joint_frames);
    const auto end_feedback = hal::decode_end_pose(pose_frames);

    // 将 HAL 的标准数组转换成运动学库需要的 Eigen 向量。
    VecXd q(6);
    for (Eigen::Index i = 0; i < q.size(); ++i)
    {
        q(i) = q_feedback[static_cast<std::size_t>(i)];
    }

    const Mat4 T_poe =
        forward_poe_space(model.S_list, q, model.M);

    // CAN 反馈末端位姿：位置 m，姿态 rad
    const Vec3 p_feedback(end_feedback.position_m[0],
                          end_feedback.position_m[1],
                          end_feedback.position_m[2]);

    // 厂家 RPY 约定：R = Rz(RZ) * Ry(RY) * Rx(RX)
    const Mat3 R_feedback =
        rot_z(end_feedback.rpy_rad[2]) *
        rot_y(end_feedback.rpy_rad[1]) *
        rot_x(end_feedback.rpy_rad[0]);

    const Mat4 T_feedback =
        se3_from_rt(R_feedback, p_feedback);

    const Vec3 p_poe = T_poe.block<3, 1>(0, 3);
    const Mat3 R_poe = T_poe.block<3, 3>(0, 0);

    const double position_error_mm =
        (p_poe - p_feedback).norm() * 1000.0;

    const Mat3 R_error = R_poe.transpose() * R_feedback;
    const double cos_angle = std::clamp(
        (R_error.trace() - 1.0) / 2.0, -1.0, 1.0);
    const double rotation_error_deg =
        std::acos(cos_angle) * 180.0 / kPi;

    std::cout << std::fixed << std::setprecision(9);
    std::cout << "Decoded q (rad): " << q.transpose() << "\n\n";
    std::cout << "T_POE:\n" << T_poe << "\n\n";
    std::cout << "T_feedback:\n" << T_feedback << "\n\n";
    std::cout << "Position error: "
              << position_error_mm << " mm\n";
    std::cout << "Rotation error: "
              << rotation_error_deg << " deg\n";

    return 0;
}
