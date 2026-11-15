// 重力计算微基准：直接用库的目标，避免手工 g++ 的 include 顺序问题
#include <chrono>
#include <cstdio>
#include <Eigen/Core>
#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/gravity.hpp"
#include "hw_arm_algorithms/model.hpp"

int main(int argc, char** argv) {
  const char* path = (argc > 1) ? argv[1] : "/tmp/hw_arm.urdf";
  hw_arm_algorithms::RobotModel model;
  if (!model.load(path)) { std::printf("加载失败: %s\n", path); return 1; }
  hw_arm_algorithms::RobotWorkspace ws(model);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq());
  Eigen::VectorXd g(model.nv());

  for (int i = 0; i < 2000; ++i) hw_arm_algorithms::computeGravity(model, ws, q, g);

  constexpr int N = 200000;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i) {
    q[1] += 1e-7;
    hw_arm_algorithms::computeGravity(model, ws, q, g);
  }
  auto t1 = std::chrono::steady_clock::now();
  const double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;

  std::printf("computeGravity: %.3f us/call\n", us);
  std::printf("  250 Hz (4000 us) → 占用 %.3f%%\n", us / 4000.0 * 100.0);
  std::printf(" 1000 Hz (1000 us) → 占用 %.3f%%\n", us / 1000.0 * 100.0);
  std::printf("g(q) = [%.3f %.3f %.3f %.3f %.3f %.3f] N·m\n",
              g[0], g[1], g[2], g[3], g[4], g[5]);
  return 0;
}
