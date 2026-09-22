#include "piper_control/robot_model/piper_model.hpp"

#include <array>
#include <cstddef>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{
namespace
{

// PiPER URDF 中一个旋转关节在零位时的参数。
struct UrdfRevoluteJointHome
{
    Vec3 xyz;   // parent link -> joint 的零位平移，单位 m。
    Vec3 rpy;   // roll, pitch, yaw，单位 rad。
    Vec3 axis;  // 关节局部坐标系下的旋转轴。
};

// URDF 的 rpy="roll pitch yaw" 对应：
//
//     R = Rz(yaw) * Ry(pitch) * Rx(roll)
//
// 返回零位时 parent link -> child link 的齐次变换。
// 对本 PiPER URDF，所有关节均为 revolute，且 q = 0，
// 因此此处不包含 Rz(q) 的关节变量旋转。
Mat4 make_urdf_home_transform(
    const UrdfRevoluteJointHome& joint)
{
    const double roll = joint.rpy.x();
    const double pitch = joint.rpy.y();
    const double yaw = joint.rpy.z();

    const Mat3 R =
        rot_z(yaw) *
        rot_y(pitch) *
        rot_x(roll);

    return se3_from_rt(R, joint.xyz);
}

}  // namespace

/*【MODEL-01】
 * @brief 创建 PiPER 六轴机械臂的 POE 运动学模型。
 *
 * 坐标系约定：
 *
 * 1. URDF 中 world_to_base_link 为单位 fixed joint，因此：
 *
 *        {world} == {base_link}
 *
 * 2. S_list 的每一列为零位时、相对 {world}/{base_link} 表达的
 *    空间旋量轴：
 *
 *        S_i = [ omega_i               ]
 *              [ -omega_i x p_i        ]
 *
 *    p_i 为第 i 根转轴上一点在 {base_link} 中的位置。
 *
 * 3. M 为零位时末端 link6 相对 {base_link} 的位姿：
 *
 *        M = T_origin_1 * ... * T_origin_6
 *
 * 4. B_list 为零位末端 {link6} 坐标系中表达的身体旋量轴：
 *
 *        B_i = Ad_(M^-1) * S_i
 */
PiperModel make_piper_model()
{
    // 这些数值直接来自 models/urdf/piper_description.urdf。
    //
    // 每个 axis 都是局部 z 轴，但经前级 rpy 累积变换后，
    // 在 {base_link} 中的空间轴方向通常不同。
    const std::array<UrdfRevoluteJointHome, kDof> joints =
    {{
        {
            Vec3(0.0, 0.0, 0.123),
            Vec3(0.0, 0.0, 0.0),
            Vec3(0.0, 0.0, 1.0)
        },
        {
            Vec3(0.0, 0.0, 0.0),
            Vec3(1.5707963, -0.1357866, -3.1415926),
            Vec3(0.0, 0.0, 1.0)
        },
        {
            Vec3(0.28503, 0.0, 0.0),
            Vec3(0.0, 0.0, -1.7938494),
            Vec3(0.0, 0.0, 1.0)
        },
        {
            Vec3(-0.02198, -0.25075, 0.0),
            Vec3(1.5707963, 0.0, 0.0),
            Vec3(0.0, 0.0, 1.0)
        },
        {
            Vec3(0.0, 0.0, 0.0),
            Vec3(-1.5707963, 0.0, 0.0),
            Vec3(0.0, 0.0, 1.0)
        },
        {
            Vec3(8.8259e-05, -0.091, 0.0),
            Vec3(1.5707963, 0.0, 0.0),
            Vec3(0.0, 0.0, 1.0)
        }
    }};

    PiperModel model;

    model.S_list = MatXd::Zero(6, kDof);

    // world_to_base_link 是单位 fixed transform：
    //
    //     T_world_base = I
    //
    // 因而这里的 T_world_parent 同时就是 T_base_parent。
    Mat4 T_world_parent = Mat4::Identity();

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const Eigen::Index column =
            static_cast<Eigen::Index>(i);

        // 零位时：
        //
        // T_world_joint_i =
        //     T_world_parent * T_parent_joint_i_home
        const Mat4 T_world_joint =
            T_world_parent *
            make_urdf_home_transform(joints[i]);

        // 关节局部轴转换到 world/base_link 坐标系。
        const Mat3 R_world_joint =
            T_world_joint.block<3, 3>(0, 0);

        const Vec3 omega_world =
            R_world_joint * joints[i].axis;

        // 转轴上一点：当前 joint 坐标系原点。
        const Vec3 point_on_axis_world =
            T_world_joint.block<3, 1>(0, 3);

        // 构造：
        //
        // S_i = [ omega ]
        //       [ -omega x p ]
        model.S_list.col(column) =
            make_revolute_screw_axis(
                omega_world,
                point_on_axis_world);

        // 零位下 child link frame 与 joint frame 重合，
        // 继续递推至下一个 joint。
        T_world_parent = T_world_joint;
    }

    // 零位末端位姿：
    //
    // M = T_world_link6(q = 0)
    model.M = T_world_parent;

    // 身体旋量：
    //
    // B_list = Ad_(M^-1) * S_list
    model.B_list =
        adjoint_inverse(model.M) *
        model.S_list;

    // 关节限制直接来自 PiPER URDF，单位均为 rad / rad/s。
    model.q_min.resize(kDof);
    model.q_max.resize(kDof);
    model.dq_max.resize(kDof);

    model.q_min <<
        -2.6179938,
         0.0,
        -2.9670597,
        -1.7453292,
        -1.2217304,
        -2.0943951;

    model.q_max <<
         2.6179938,
         3.1415926,
         0.0,
         1.7453292,
         1.2217304,
         2.0943951;

    model.dq_max <<
        5.0,
        5.0,
        5.0,
        5.0,
        5.0,
        5.0;

    return model;
}

}  // namespace piper_control