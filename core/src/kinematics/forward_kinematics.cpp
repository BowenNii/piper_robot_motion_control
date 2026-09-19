#include "piper_control/kinematics/forward_kinematics.hpp"

#include <cassert>

#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{

/*【FK-01】
 * @brief 使用 POE 空间形式计算正运动学。
 */
Mat4 forward_poe(
    const MatXd& S_list,
    const VecXd& q,
    const Mat4& M)
{
    assert(S_list.rows() == 6);
    assert(S_list.cols() == q.size());

    Mat4 T = Mat4::Identity();

    for (Eigen::Index i = 0; i < q.size(); ++i)
    {
        T = T * screw_exp(S_list.col(i), q(i));
    }

    return T * M;
}

}  // namespace piper_control