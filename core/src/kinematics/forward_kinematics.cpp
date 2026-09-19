#include "piper_control/kinematics/forward_kinematics.hpp"

#include <cassert>

#include "piper_control/lie_group/screw.hpp"

namespace piper_control
{

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
        const Twist S = S_list.col(i);

        T = T * screw_exp(S, q(i));
    }

    // 标准空间 POE：
    // T(q) = exp(S1 q1) ... exp(Sn qn) M
    return T * M;
}

}  // namespace piper_control