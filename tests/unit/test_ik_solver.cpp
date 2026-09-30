#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
using namespace piper_control;
int main() {
  bool pass = true;
  auto check = [&](bool ok, const char *name) {
    if (!ok)
      std::cerr << "[FAIL] " << name << '\n';
    pass &= ok;
  };
  using Solver = IKResult (*)(const MatXd &, const Mat4 &, const Mat4 &,
                              const VecXd &, const IKOptions &);
  const Solver solvers[] = {solve_ik_newton, solve_ik_dls,
                            solve_ik_adaptive_dls, solve_ik_transpose};
  // 一移动关节，有独立解析答案x=q；六维任务Jacobian秩不足。
  MatXd S = MatXd::Zero(6, 1);
  S(3, 0) = 1;
  const Mat4 M = Mat4::Identity();
  Mat4 target = M;
  target(0, 3) = 0.4;
  for (auto solver : solvers) {
    IKOptions o;
    o.max_iterations = 300;
    auto r = solver(S, M, target, VecXd::Zero(1), o);
    check(r.success && std::abs(r.q(0) - 0.4) < 1e-6,
          "analytic prismatic solution");
    r = solver(S, M, M, VecXd::Zero(1), o);
    check(r.success && r.iterations == 0, "already at target");
    Mat4 impossible = M;
    impossible(1, 3) = 1;
    r = solver(S, M, impossible, VecXd::Zero(1), o);
    check(!r.success && r.q.allFinite() && r.position_error > 0.9,
          "unreachable finite failure");
    o.q_min = VecXd::Constant(1, -0.1);
    o.q_max = VecXd::Constant(1, 0.1);
    r = solver(S, M, target, VecXd::Zero(1), o);
    check(!r.success && r.q(0) <= 0.1 && r.q(0) >= -0.1, "bounds enforced");
    o = IKOptions{};
    o.max_iterations = 0;
    r = solver(S, M, target, VecXd::Zero(1), o);
    check(!r.success && r.status == IKStatus::kMaxIterations,
          "iteration limit");
    Mat4 invalid = M;
    invalid(3, 3) = 0;
    bool caught = false;
    try {
      (void)solver(S, M, invalid, VecXd::Zero(1), o);
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    check(caught, "invalid pose rejected");
    caught = false;
    o.lambda = std::numeric_limits<double>::quiet_NaN();
    try {
      (void)solver(S, M, target, VecXd::Zero(1), o);
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    check(caught, "NaN options rejected");
  }
  const auto model = make_piper_model();
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform(0, 1);
  // PiPER：限位内目标，从邻近初值恢复末端位姿；不要求恢复同一组q。
  for (int method = 0; method < 4; ++method) {
    rng.seed(42); // 各方法使用同一批目标和初值，转置法取前5组。
    int successes = 0;
    const int count = method == 3 ? 5 : 50;
    for (int sample = 0; sample < count; ++sample) {
      VecXd goal(6), initial(6);
      for (int i = 0; i < 6; ++i) {
        goal(i) = model.q_min(i) + (0.2 + 0.6 * uniform(rng)) *
                                       (model.q_max(i) - model.q_min(i));
        initial(i) = goal(i) + 0.04 * (uniform(rng) - 0.5);
      }
      const Mat4 desired = forward_poe_space(model.S_list, goal, model.M);
      IKOptions o;
      o.q_min = model.q_min;
      o.q_max = model.q_max;
      o.max_iterations = method == 3 ? 5000 : 500;
      const auto r =
          solvers[method](model.S_list, model.M, desired, initial, o);
      check(r.q.allFinite() && (r.q.array() >= o.q_min.array()).all() &&
                (r.q.array() <= o.q_max.array()).all(),
            "PiPER bounds and finite values");
      if (r.success) {
        ++successes;
        const Mat4 actual = forward_poe_space(model.S_list, r.q, model.M);
        check((actual.topRightCorner<3, 1>() - desired.topRightCorner<3, 1>())
                      .norm() <= 1e-6,
              "independent FK position residual");
        check((actual.topLeftCorner<3, 3>() - desired.topLeftCorner<3, 3>())
                      .norm() < 2e-6,
              "independent FK rotation residual");
      }
    }
    std::cout << "method " << method << ": " << successes << '/' << count
              << " converged\n";
    // 转置法只记录PiPER表现，不把局部不收敛误称为算法错误。
    if (method < 2)
      check(successes == count, "Newton/DLS nearby PiPER convergence");
    if (method == 2)
      check(successes > 0, "adaptive PiPER convergence");
  }
  return pass ? 0 : 1;
}
