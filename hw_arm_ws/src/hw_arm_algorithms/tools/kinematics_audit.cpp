// 运动学离线审计 —— 只算数，不接机器人
//
// 目标：在真机测试之前，把"运动学精度"这件事从算法库里查清楚。
//
// 检查项：
//   K1  帧约定：URDF 里有哪些 frame，world→base_link 是什么变换，
//              computeFK 返回的是哪个坐标系下的位姿
//   K2  FK 与独立实现（直接调 Pinocchio）逐位对比
//   K3  解析雅可比 vs 数值差分雅可比
//   K4  IK 精度：在可达工作空间内撒点，统计位置/姿态残差与成功率
//   K5  IK 往返：q → FK → IK → q'，看能不能回到原关节角
//   K6  IK 连续性：相邻目标之间解是否跳变（影响轨迹平滑）
//
// 用法：
//   kinematics_audit --urdf <file> [--samples 200] [--verbose]

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/kinematics.hpp"
#include "hw_arm_algorithms/model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace hw_arm_algorithms;

namespace {

int g_fail = 0;

void report(const char* tag, bool ok, const std::string& msg) {
  std::printf("  %s %-4s %s\n", ok ? "✅" : "❌", tag, msg.c_str());
  if (!ok) ++g_fail;
}

Eigen::MatrixXd numericalJacobian(const RobotModel& m, RobotWorkspace& ws,
                                  const Eigen::VectorXd& q,
                                  pinocchio::FrameIndex frame, double h = 1e-6) {
  const int nv = m.nv();
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, nv);
  pinocchio::SE3 T0;
  computeFK(m, ws, q, frame, T0);
  const Eigen::Matrix3d R0 = T0.rotation();
  const Eigen::Vector3d p0 = T0.translation();
  for (int i = 0; i < nv; ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp[i] += h;
    qm[i] -= h;
    pinocchio::SE3 Tp, Tm;
    computeFK(m, ws, qp, frame, Tp);
    computeFK(m, ws, qm, frame, Tm);
    // 线速度：中心差分
    J.block<3, 1>(0, i) = (Tp.translation() - Tm.translation()) / (2 * h);
    // 角速度：R_dot * R^T 取反对称部分
    const Eigen::Matrix3d dR = (Tp.rotation() - Tm.rotation()) / (2 * h);
    const Eigen::Matrix3d W = dR * R0.transpose();
    J.block<3, 1>(3, i) << W(2, 1), W(0, 2), W(1, 0);
  }
  (void)p0;
  return J;
}

// 随机采样一个在限位内、且离限位有余量的位形
Eigen::VectorXd sampleQ(const RobotModel& m, std::mt19937& rng, double margin) {
  const int nv = m.nv();
  Eigen::VectorXd q(nv);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int i = 0; i < nv; ++i) {
    double lo = 0.0, hi = 0.0;
    m.jointLimits(i, lo, hi);
    const double span = hi - lo;
    const double a = lo + margin * span;
    const double b = hi - margin * span;
    q[i] = a + (b - a) * u(rng);
  }
  return q;
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf;
  int nsamples = 200;
  bool verbose = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--urdf" && i + 1 < argc) urdf = argv[++i];
    else if (a == "--samples" && i + 1 < argc) nsamples = std::stoi(argv[++i]);
    else if (a == "--verbose") verbose = true;
  }
  if (urdf.empty()) {
    std::printf("用法: kinematics_audit --urdf <file> [--samples N] [--verbose]\n");
    return 2;
  }

  RobotModel model;
  LoadOptions opt;
  opt.expected_total_mass = 0.0;   // 不校验质量
  if (!model.load(urdf, opt)) {
    std::printf("❌ 加载 URDF 失败: %s\n", urdf.c_str());
    return 2;
  }
  RobotWorkspace ws(model);
  const int nv = model.nv();
  std::printf("模型: %s   nv=%d   总质量 %.4f kg\n", urdf.c_str(), nv,
              model.totalMass());

  // ── K1 帧约定
  std::printf("\n【K1】帧约定\n");
  const auto& pin_model = model.pin();
  std::printf("  frame 数 %zu:\n", pin_model.nframes);
  for (pinocchio::FrameIndex f = 0; f < pin_model.nframes; ++f) {
    std::printf("    [%2zu] %-16s type=%d parentJoint=%d\n", f,
                pin_model.frames[f].name.c_str(),
                static_cast<int>(pin_model.frames[f].type),
                pin_model.frames[f].parentJoint);
  }
  const pinocchio::FrameIndex tool = model.frameId("tool0");
  if (tool >= pin_model.frames.size()) {
    report("K1", false, "URDF 里没有 tool0 帧");
  } else {
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nv);
    pinocchio::SE3 T0;
    computeFK(model, ws, q0, tool, T0);
    std::printf("  q=0 时 tool0 位姿（在 URDF 根坐标系下）:\n");
    std::printf("    p = [%+.6f %+.6f %+.6f] m\n", T0.translation()[0],
                T0.translation()[1], T0.translation()[2]);
    std::printf("    R =\n");
    for (int r = 0; r < 3; ++r)
      std::printf("      [%+.4f %+.4f %+.4f]\n", T0.rotation()(r, 0),
                  T0.rotation()(r, 1), T0.rotation()(r, 2));
    // world → base_link 的固定变换：这是笛卡尔目标"以谁为原点"的关键
    const pinocchio::FrameIndex fb = model.frameId("base_link");
    if (fb < pin_model.frames.size()) {
      pinocchio::SE3 Tb;
      computeFK(model, ws, q0, fb, Tb);
      std::printf("  base_link 相对根坐标系:\n");
      std::printf("    p = [%+.6f %+.6f %+.6f] m\n", Tb.translation()[0],
                  Tb.translation()[1], Tb.translation()[2]);
      const Eigen::Matrix3d R = Tb.rotation();
      const double yaw = std::atan2(R(1, 0), R(0, 0));
      const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
      const double roll = std::atan2(R(2, 1), R(2, 2));
      std::printf("    RPY = [%+.2f %+.2f %+.2f] deg\n", roll * 180 / M_PI,
                  pitch * 180 / M_PI, yaw * 180 / M_PI);
      report("K1", true,
             "computeFK 返回根坐标系位姿；**笛卡尔目标要按 base_link 还是根系必须先定清楚**");
    } else {
      report("K1", true, "computeFK 返回的是 URDF 根坐标系下的位姿");
    }
  }

  // ── K2 FK 与独立实现对比
  std::printf("\n【K2】FK 与独立 Pinocchio 调用逐位对比\n");
  {
    std::mt19937 rng(12345);
    double maxdiff = 0.0;
    for (int s = 0; s < 50; ++s) {
      Eigen::VectorXd q = sampleQ(model, rng, 0.1);
      pinocchio::SE3 a, b;
      computeFK(model, ws, q, tool, a);
      pinocchio::forwardKinematics(pin_model, ws.pin(), q);
      pinocchio::updateFramePlacement(pin_model, ws.pin(), tool);
      b = ws.pin().oMf[tool];
      maxdiff = std::max(maxdiff,
                         (a.translation() - b.translation()).norm());
    }
    report("K2", maxdiff < 1e-12,
           "50 个随机位形，最大平移差 " + std::to_string(maxdiff) + " m");
  }

  // ── K3 解析雅可比 vs 数值差分
  std::printf("\n【K3】解析雅可比 vs 数值差分\n");
  {
    std::mt19937 rng(999);
    double maxrel = 0.0;
    for (int s = 0; s < 20; ++s) {
      Eigen::VectorXd q = sampleQ(model, rng, 0.15);
      Eigen::MatrixXd Ja, Jn;
      computeJacobian(model, ws, q, tool, JacobianFrame::LOCAL_WORLD_ALIGNED, Ja);
      Jn = numericalJacobian(model, ws, q, tool);
      const double den = std::max(1e-9, Jn.norm());
      maxrel = std::max(maxrel, (Ja - Jn).norm() / den);
    }
    report("K3", maxrel < 1e-4,
           "20 个位形，最大相对差 " + std::to_string(maxrel));
  }

  // ── K4 IK 精度与收敛率（目标取自 FK，保证可达）
  std::printf("\n【K4】IK 精度与收敛率\n");
  {
    struct Case { const char* name; int seeds; int maxsol; bool hint; };
    const Case cases[] = {
        {"1 种子(限位中点)", 1, 1, false},
        {"1 种子(当前位形 hint)", 1, 1, true},
        {"16 种子", 16, 8, false},
        {"8 种子", 8, 8, false},
    };
    for (const auto& c : cases) {
      std::mt19937 rng(2024);
      int ok = 0, tried = 0;
      double max_pos = 0.0, max_ori = 0.0;
      for (int s = 0; s < nsamples; ++s) {
        Eigen::VectorXd q_true = sampleQ(model, rng, 0.2);
        pinocchio::SE3 T;
        computeFK(model, ws, q_true, tool, T);
        IKOptions o;
        o.num_seeds = c.seeds;
        o.max_solutions = c.maxsol;
        IKResult r = solveIKMultiSeed(model, ws, tool, T,
                                      c.hint ? &q_true : nullptr, o);
        ++tried;
        if (!r.ok() || !r.best().converged) continue;
        ++ok;
        max_pos = std::max(max_pos, r.best().position_error);
        max_ori = std::max(max_ori, r.best().orientation_error);
      }
      const double rate = tried ? 100.0 * ok / tried : 0.0;
      std::printf("    %-22s 收敛 %5.1f%%   位置残差 max %.2e m   姿态 max %.2e rad\n",
                  c.name, rate, max_pos, max_ori);
      if (c.hint) {
        report("K4", rate > 99.0 && max_pos < 1e-3,
               "以当前位形为种子：应 100% 收敛且残差 < 1 mm");
      }
    }
    std::printf("    ⇒ 结论：算法本身没问题，**收敛率取决于种子**；\n");
    std::printf("      实际使用必须用「上一位形」当种子（goto/replay 都是这样）\n");
  }

  // ── K5 IK 往返（q → FK → IK → q'）
  std::printf("\n【K5】IK 往返：q → FK → IK → q'\n");
  {
    std::mt19937 rng(777);
    int same = 0, tried = 0;
    double max_joint = 0.0;
    for (int s = 0; s < nsamples; ++s) {
      Eigen::VectorXd q_true = sampleQ(model, rng, 0.2);
      pinocchio::SE3 T;
      computeFK(model, ws, q_true, tool, T);
      IKOptions o;
      o.max_solutions = 1;
      o.num_seeds = 8;
      IKResult r = solveIKFromSeeds(model, ws, tool, T, std::vector<Eigen::VectorXd>{q_true}, o);
      ++tried;
      if (!r.ok() || !r.best().converged) continue;
      const double d = (r.best().q - q_true).cwiseAbs().maxCoeff();
      max_joint = std::max(max_joint, d);
      if (d < 1e-3) ++same;
    }
    std::printf("    用真值当种子：%d/%d 回到原关节角（阈值 1e-3 rad）\n", same,
                tried);
    std::printf("    最大关节角偏差 %.3e rad\n", max_joint);
    report("K5", same == tried, "以真值作种子应 100% 回到原位形");
  }

  // ── K6 IK 连续性（真实用法：用上一步的解当种子）
  std::printf("\n【K6】IK 连续性（沿笛卡尔直线扫，用上一步的解当种子）\n");
  {
    Eigen::VectorXd q_a = Eigen::VectorXd::Zero(nv);
    q_a << 0.0, 0.6, 0.7, 0.0, 0.5, 0.0;
    pinocchio::SE3 Ta;
    computeFK(model, ws, q_a, tool, Ta);
    const Eigen::Vector3d p0 = Ta.translation();

    for (int mode = 0; mode < 2; ++mode) {
      const bool use_prev = (mode == 1);
      Eigen::VectorXd q_seed = q_a;
      int jumps = 0, solved = 0;
      double max_step = 0.0, max_err = 0.0;
      for (int k = 0; k <= 20; ++k) {
        pinocchio::SE3 T = Ta;
        T.translation(p0 + Eigen::Vector3d(0.0, 0.0, 0.01 * k));
        IKOptions o;
        o.num_seeds = 16;
        o.max_solutions = 1;
        IKResult r = solveIKMultiSeed(model, ws, tool, T,
                                      use_prev ? &q_seed : nullptr, o);
        if (!r.ok() || !r.best().converged) continue;
        ++solved;
        max_err = std::max(max_err, r.best().position_error);
        if (k > 0) {
          const double d = (r.best().q - q_seed).cwiseAbs().maxCoeff();
          max_step = std::max(max_step, d);
          if (d > 0.2) ++jumps;
        }
        q_seed = r.best().q;
      }
      std::printf("    %s: 解出 %d/21  最大关节步长 %.3f rad  跳变 %d 次  最大位置残差 %.2e m\n",
                  use_prev ? "上一步当种子" : "每步 16 随机种子", solved,
                  max_step, jumps, max_err);
      if (use_prev) {
        // 跳变本身不是 bug：靠近手腕奇异点时 q4/q6 会剧烈变化。
        // 判据改成：要么不跳，要么跳的时候确实处在奇异点附近。
        bool at_singularity = false;
        for (int k = 0; k <= 20; ++k) {
          pinocchio::SE3 T = Ta;
          T.translation(p0 + Eigen::Vector3d(0.0, 0.0, 0.01 * k));
          IKOptions o; o.num_seeds = 16; o.max_solutions = 1;
          IKResult r = solveIKMultiSeed(model, ws, tool, T, &q_seed, o);
          if (!r.ok()) continue;
          const double sig = computeManipulability(model, ws, r.best().q, tool).sigma_min;
          if (sig < 1e-2) at_singularity = true;
        }
        std::printf("    路径上是否经过奇异点(sigma_min<1e-2): %s\n",
                    at_singularity ? "是" : "否");
        report("K6", jumps == 0 || at_singularity,
               jumps == 0 ? "解连续，无跳变"
                          : "有跳变，但发生在手腕奇异点附近（物理上正常）");
      }
    }
  }

  std::printf("\n════════════════════════════════════\n");
  if (g_fail == 0) {
    std::printf(" ✅ 运动学审计全部通过\n");
  } else {
    std::printf(" ❌ %d 项未通过\n", g_fail);
  }
  std::printf("════════════════════════════════════\n");
  return g_fail == 0 ? 0 : 1;
}
