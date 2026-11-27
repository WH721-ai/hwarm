# 开源选题：这个领域真正缺的是什么

> 结论先行：**"URDF → MCU 可用的零依赖 C 代码（含动力学，不只是 FK/碰撞）" 目前是生态空白**。
> 我们做的这个项目，本身就是一个可以发出去的开源件——不是"顺手开源"，而是补位。

---

## 1. 生态盘点（2026-10）

| 项目 | 做了什么 | 为什么不能直接满足"MCU 上的机械臂算法库" |
|---|---|---|
| **Pinocchio** | 完整的刚体动力学 + 解析导数 + 标量模板 | 依赖 Eigen / Boost / urdfdom / 动态分配，**没有任何 MCU 后端**；它是"宿主端编译器前端" |
| **CppADCodeGen** | 通用 AD → C 代码生成 | 是**通用工具**，没有机器人语义：没有 URDF 输入、没有 FK/RNEA 模板、没有 MCU 运行时的接口约定 |
| **Cricket (CoMMALab)** | URDF → C++ 头文件（FK、球体碰撞、ABA…） | **VAMP 专用**：生成物 `#include <vamp/vector.hh>`、依赖 `FloatVector` SIMD、`vamp/collision/*`；接口是 `vamp::robots::X`，不是通用库 |
| **VAMP** | x86 AVX2 并行规划器 | 面向桌面 CPU 的 SIMD，与 Cortex-M 无关 |
| **RBDL / DART / KDL** | 动力学库 | 无 AD、无代码生成（教程 M02 §5.1 的功能矩阵里"代码生成"一行只有 Pinocchio 打勾） |
| **Drake** | AutoDiffXd 全栈 | 导不出 C 代码；Bazel + 巨型依赖 |
| **SimpleFOC / ODrive / Teensy 库** | MCU 上的电机控制 / 手写 IK | 手工建模、无 URDF 链路、无动力学 |
| **OSQP/ProxQP/HQIPM** | 求解器 | 面向 PC，堆分配 + 动态尺寸 |

**空白点归纳成一句话**：从 URDF 到"Cortex-M 上能跑的零依赖 C 函数（FK/雅可比/RNEA/重力/ABA），带精度与周期双验收"，中间没有任何一个成熟开源项目。

---

## 2. 四个候选项目（按"价值 × 独特性 ÷ 难度"排序）

| # | 项目 | 一句话 | 独特性 | 难度 | 复用本项目 |
|---|---|---|---|---|---|
| 1 | **`urdf2mcu`**（生成器） | 任意 URDF → 纯 C99 的机械臂运动学/动力学库 | ★★★★★ | 中 | 100% |
| 2 | `libarmkin`（运行时 + 基准） | 零堆、定长、WCET 可静态分析的 MCU 侧运行时 | ★★★★ | 中 | 100%（可作为 1 的产物） |
| 3 | `pinocchio-golden` | 给定 URDF 生成 1e4 组黄金参考 + 对拍报告 | ★★★ | 低 | 80% |
| 4 | `mcu-kin-bench` | H723/H743/Teensy 上 FK/RNEA/ABA 的公开基准 | ★★★★ | 低 | 60% |

**建议**：把 1 + 2 合成一个仓库发（生成器 + 运行时），3 作为它的测试子命令，4 作为 `benchmark/` 目录 + README 表格。这样一次就把空白填了。

---

## 3. `urdf2mcu` 设计草案

```
urdf2mcu/
├── src/                     # 宿主侧生成器（C++17）
│   ├── main.cc              #   CLI: urdf2mcu --urdf r.urdf --ee tool0 --out gen/ --lang c --scalar float
│   ├── trace.{hh,cc}        #   Pinocchio + CppAD 追踪（FK / Jacobian / RNEA / g / ABA / collision）
│   ├── emit_c.{hh,cc}       #   CppADCodeGen LanguageC<float|double> + 函数切分
│   └── render.{hh,cc}       #   inja 模板 → 头文件 + 元数据 json
├── templates/               # C99 模板（不含任何机器人常量）
├── runtime/                 # 目标端零依赖运行时（C99）
│   ├── include/armkin/*.h
│   └── src/{dls.c, limit.c, math.c}
├── tests/                   # 对拍（Pinocchio double）与回归
├── benchmark/               # 节点周期 + 体积，按 MCU 型号分表
├── examples/                # Panda / UR5 / 自定义臂 / Keil 工程模板
└── cmake/                   # toolchain 文件 + 安装规则
```

**三条差异化（相对 Cricket）**
1. **通用**：不绑定任何规划器；生成物只依赖 `<math.h>`，可嵌进 Keil / CubeMX / Zephyr / 裸机任意工程。
2. **动力学全覆盖**：FK / 雅可比 / RNEA / 重力 / ABA 都生成（Cricket 主打 FK 与球体碰撞），并支持按需裁剪（`--enable rnea,gravity`）。
3. **自带双重验收**：每个函数都有 golden 对拍（精度）+ DWT 周期基准（实时性），CI 全自动。

**MVP 边界（第一版只做这些）**
- 输入：单个 URDF + 末端 frame 名
- 输出：`arm_<name>_{fk,jac,rnea,g,aba}.h` + `arm_<name>_meta.json`
- 标量：float（默认）/ double（`--scalar double`）
- 不含：闭链、浮动基座、球体碰撞、QP

**验收（README 第一屏就写）**

| 机器人 | FK 误差 | 雅可比误差 | 力矩误差 | FK 周期 | RNEA 周期 |
|---|---|---|---|---|---|
| Panda (7) | ≤ 1e-5 m | ≤ 1e-5 | ≤ 1e-3 N·m | ≤ 3 µs | ≤ 6 µs |

（周期在 STM32H723 @550 MHz、float、`-O3` 下测）

---

## 4. 三个 sprint 的发布路线

| Sprint | 内容 | 可发布？ |
|---|---|---|
| **S1**（2 周） | 生成器 FK/雅可比 + 对拍 CI + README | 可以发 v0.1（"能生成 FK 的 codegen"，已超过多数玩具库） |
| **S2**（2 周） | RNEA / 重力 / ABA + 体积/栈报告 + benchmark 表 | 发 v0.2，写一篇技术博客（"MCU 上的解析动力学：体积换周期"） |
| **S3**（1–2 周） | DLS-IK 运行时 + 实机案例（DM-MC-Board02）+ Keil 模板 | 发 v1.0，投 ROS Discourse / HN / r/robotics |

**发布前置检查**
- [ ] `git init` + 干净的提交历史（当前工作区还不是 git 仓库）
- [ ] LICENSE 文件（建议 BSD-3-Clause，与 Pinocchio 兼容、工业友好）
- [ ] CI：GitHub Actions（x86 生成 + 对拍） + 可选 ARM 交叉编译与 QEMU 冒烟
- [ ] `examples/` 能在 3 分钟内跑通（`git clone` → `cmake` → `make` → 看到生成的 `.h`）
- [ ] README 里放 benchmark 表 + 一张"体积 vs 周期"的图

---

## 5. 许可与合规（必须先搞清楚再发）

| 依赖 | 许可 | 影响 |
|---|---|---|
| Pinocchio | BSD-2 | ✅ 可静态链接、可商用 |
| Eigen | MPL-2.0 | ✅ 仅头文件 |
| CppAD | EPL-1.0 / GPL-3.0 双许可 | ✅ 用 EPL 分支即可 |
| **CppADCodeGen** | **EPL-1.0 / GPL-3.0 双许可** | ⚠️ 它是**生成器**：我们的产物是它吐出的 C 代码。业界（含 Cricket/VAMP 生态）的实践是"生成物不含其运行时头，故不受 copyleft 约束"。**但必须在 README 明确声明**，并保证：① 生成器是可选依赖（用户可自带 `.h`）；② 生成的 C 不 `#include` 任何 `cppad/cg` 头。 |
| 我们的产物 | BSD-3 | 与 Pinocchio 一致，工业友好 |

**动作**：README 加一节 "Generated code is free of runtime dependencies and contains no CppADCodeGen headers; the generator itself is an optional build-time dependency."

---

## 6. 传播渠道（按性价比）

1. **达妙社区 / 教程作者**：教程 M06 正好讲这块，作者（Pengfei Guo，达妙科技）是天然的第一位审阅者，也是最好的背书。
2. **ROS Discourse**（`ros2_control` + 嵌入式板块）：发"URDF → MCU"的完整链路。
3. **Hacker News / r/robotics**：标题走"把 Pinocchio 编译进 3 美元的 MCU"。
4. **中文**：知乎 / 古月居 / B 站（配一段串口打印 + 实机动图）。
5. **awesome-robotics / awesome-embedded-cpp** 提 PR。

---

## 7. 与本项目的关系

```
本项目（私有/自用）              开源件
─────────────────────────────────────────────
armgen 生成器          ──抽出来→   urdf2mcu
runtime/dls.c + 生成物 ──打包→     libarmkin
host/tests/golden     ──独立→     pinocchio-golden（子命令）
板卡实测数据           ──公开→     benchmark/ 表格
实机联调记录           ──脱敏→     examples/ + 博客
```

**不需要"另做一份"**：从第一天就按开源布局建目录（`src/ templates/ runtime/ tests/ benchmark/`），项目做完 = 仓库可以发。这也是我在 `PLAN.md` §6 里已经按这个结构规划的原因。
