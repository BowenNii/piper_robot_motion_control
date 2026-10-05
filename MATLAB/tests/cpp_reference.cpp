// 离线交叉验证辅助程序：调用实际 C++ 库，输出数值，不发送 CAN。
#include <iomanip>
#include <iostream>
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/kinematics/singularity.hpp"
#include "piper_control/control/cartesian_velocity_controller.hpp"

using namespace piper_control;
int main() {
  std::cout << std::setprecision(17);
  const auto model = make_piper_model();
  VecXd q(6), goal(6);
  q << .2, 1., -1., .3, -.2, .4;
  goal = q;
  goal += (VecXd(6) << .02, -.01, .01, .01, -.01, .01).finished();
  const auto target = forward_poe_space(model.S_list, goal, model.M);
  for (int row=0;row<4;++row)
    for (int col=0;col<4;++col) std::cout << target(row,col) << ' ';
  IKOptions o;
  o.max_iterations=100;
  const IKResult results[] = {
    solve_ik_newton(model.S_list,model.M,target,q,o),
    solve_ik_dls(model.S_list,model.M,target,q,o),
    solve_ik_adaptive_dls(model.S_list,model.M,target,q,o),
    solve_ik_transpose(model.S_list,model.M,target,q,o)};
  for (const auto& r:results) {
    std::cout << r.success << ' ' << r.iterations << ' '
              << static_cast<int>(r.status) << ' '
              << r.position_error << ' ' << r.orientation_error << ' ';
    for (int i=0;i<6;++i) std::cout << r.q(i) << ' ';
  }
  MatXd J(2,4); J << 1,2,0,0,0,1,3,0;
  const auto P=pseudoinverse_svd(J,1e-12);
  for (int row=0;row<4;++row)
    for (int col=0;col<2;++col) std::cout << P(row,col) << ' ';
  MatXd A=MatXd::Identity(6,6); A(5,5)=.025;
  Vec6 v=Vec6::Ones();
  const auto velocity=solve_cartesian_velocity(A,v,CartesianVelocityControllerConfig{});
  for (int i=0;i<6;++i) std::cout << velocity.q_dot_command(i) << ' ';
  std::cout << velocity.sigma_min << ' ' << velocity.speed_scale << ' '
            << static_cast<int>(velocity.region) << '\n';
}
