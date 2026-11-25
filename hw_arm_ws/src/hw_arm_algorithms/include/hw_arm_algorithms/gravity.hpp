// Copyright 2026 hw_arm
// Licensed under the Apache License, Version 2.0
//
// 重力项计算
//
//   数学：g(q) = RNEA(q, 0, 0)   —— 零速零加速度时逆动力学只剩重力项
//   物理：让机械臂在姿态 q 下**静止不动**，各关节需要输出的力矩
//
// 用途：填入 MIT 控制帧的 tau_ff，消除重力造成的稳态下垂
//   无补偿：kp·e = g  →  e = g/kp   （joint2: 5.70/70 = 0.0815 rad = 4.7°）
//   有补偿：e → 0（只剩摩擦死区）
//
// 本文件刻意**不含**惯量项和科氏项。完整逆动力学等做力控时再并入
// dynamics.hpp（见 algo/docs/90_附录/Appendix_C 的 C-16）。
#pragma once

#include <vector>

#include <Eigen/Core>

#include "hw_arm_algorithms/data.hpp"
#include "hw_arm_algorithms/model.hpp"

namespace hw_arm_algorithms {

/// 计算重力补偿力矩（URDF 关节坐标系）
///
/// @param model  已加载的模型
/// @param ws     工作区（内部 Data 会被写入）
/// @param q      当前关节角（URDF 坐标，长度 nq）
/// @param g_out  输出：各关节力矩 N·m（长度 nv）
///
/// @note 不做堆分配；g_out 必须预先 resize 到 nv
void computeGravity(const RobotModel& model, RobotWorkspace& ws,
                    const Eigen::VectorXd& q, Eigen::VectorXd& g_out);

/// 计算重力补偿力矩（电机坐标系，可直接填 tau_ff）
///
/// 处理「电机转向与 URDF 关节轴相反」的情况（对应待办 C-02）：
///   tau_ff_motor[i] = direction[i] * g_urdf[i]
///
/// @param direction  每轴 ±1，长度 nv
/// @param tau_ff_out 输出：电机坐标系下的前馈力矩
void computeGravityMotor(const RobotModel& model, RobotWorkspace& ws,
                         const Eigen::VectorXd& q,
                         const std::vector<double>& direction,
                         Eigen::VectorXd& tau_ff_out);

/// 交叉验证两种算法是否一致：
///   pinocchio::computeGeneralizedGravity(model, data, q)
///   pinocchio::rnea(model, data, q, 0, 0)
///
/// @return 两者的最大绝对偏差（应 < 1e-10）
double verifyGravityConsistency(const RobotModel& model, RobotWorkspace& ws,
                                const Eigen::VectorXd& q);

/// 重力回归矩阵：g(q) = Y_g(q) · θ
///
/// θ 的排列与 Pinocchio 的 DynamicParameters 一致：
///   每个 body 10 个参数 [m, m·cx, m·cy, m·cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]
///   全部 body 拼起来长度 = 10 × (njoints − 1)
///
/// ⚠️ 对本臂：60 列里只有 24 列（每 body 前 4 个）非零，
///    这 24 列的**有效秩是 10**（实测），所以只有 10 个独立组合可辨识。
///    但 g(q) 落在秩 10 的子空间里 ⇒ 参数辨识仍能**精确**复现重力矩。
///
/// @param Y_out  输出 nv × (10*(njoints-1))，必须预先 resize
void computeGravityRegressor(const RobotModel& model, RobotWorkspace& ws,
                             const Eigen::VectorXd& q, Eigen::MatrixXd& Y_out);

/// 用辨识出来的参数算重力：g = Y_g(q)·θ
///
/// 与 computeGravity（用 URDF 里的标称惯性参数）的区别：
///   computeGravity            → g = Y·θ_nominal
///   computeGravityFromParams  → g = Y·θ_identified
/// 后者是 C-05 静态重力辨识的产物，能吸收夹爪质量、线束、装配偏差。
void computeGravityFromParams(const RobotModel& model, RobotWorkspace& ws,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& theta,
                              Eigen::VectorXd& g_out);

/// 回归矩阵的列数（= 10 × (njoints − 1)）
int gravityParamCount(const RobotModel& model);

}  // namespace hw_arm_algorithms
