# 项目驱动的学习路径（12 周）

> 目标：**边做项目边把达妙《Robotics Tutorial》机械臂方向的干货吃掉**，每个学习单元都产出一个可验收的项目件。
> 原则：① 只学这个项目用得上的；② 每个单元必须有可测的验收指标；③ 学完的东西回填进 skill（`skills/robotics-motion-control/`）。

---

> **路线说明**：本路径假设"控制跑在 MCU"。若先走 PC 路线（`ARCHITECTURE-OPTIONS.md` 的 P1），
> 第 7–8 周会被提前到最前面：先用 `ros2_control` + CANFD 让机械臂动起来（学 M12/M14/M15），
> 再回来做 M1–M4 的代码生成（学 M01/M03/M06），最后 P3 下沉 MCU。两条路共用同一份生成代码。

## 0. 总览

| 周 | 学（教程章节） | 做（项目任务） | 验收（可测） |
|---|---|---|---|
| 0 ✅ | — | M0：工具链 + URDF + 串口 demo | 串口出字 |
| 1 | M01 §1–§5（Model/Data、CRTP、标量参数化） | M1：armgen 骨架，生成 `fk_ee` | 1e4 组 q，位置误差 < 1e-5 m |
| 2 | M01 §6–§7（雅可比三参考系） + M06 §2 | M1：生成 `jacobian`（LOCAL_WORLD_ALIGNED） | 雅可比逐元素 < 1e-5 |
| 3 | M06 全篇（tape、CodeGen、开销、体积） | M2：切分函数、`-fstack-usage`、体积报告 | Flash ≤ 150 KB，栈帧 ≤ 2 KB |
| 4 | M03 §1–§3（IK 谱系、DLS） | M3：DLS 迭代外壳 + 6×6 LDLT | 1000 目标点 P99 ≤ 0.05 mm |
| 5 | M03 §4–§8（自适应 λ、零空间、选解） | M3：λ 自适应 + 限位 + 失败回退 | 奇异位形不发散，超时回退 |
| 6 | M01 §8–§10（RNEA/ABA/CRBA） | M4：`rnea` / `gravity` / `aba` 生成 | 力矩 < 1e-3 N·m，栈实测 |
| 7 | M11 全篇（实时铁律）+ M06 §预算 | M5：1 kHz 主循环 + DWT 计时 | 整周期 ≤ 300 µs，无堆符号 |
| 8 | M12 全篇（ros2_control、驱动器） | M5：驱动器状态机 + 上电自检 | 不飞车，自检 PASS |
| 9 | P01 全篇（URDF/惯量/单位） | 实机 URDF 接入，重跑对拍 | 全链路误差 ≤ 0.1 mm |
| 10 | M04 §4–§7（胶囊、距离梯度） | 碰撞监控（可选模块） | 单周期 +≤ 10 µs |
| 11 | M15（联调分层排查）+ M02（选型） | 开源化：仓库整理、CI、benchmark | CI 绿，benchmark 表 |
| 12 | 回顾 + 补漏 | 发布 + 写一篇工程博客 | 3 个外部用户试用 |

**工时**：业余 10–15 h/周 ≈ 12 周；全职 ≈ 5 周。时间紧张走 §4 的 6 周最小路径。

---

## 1. 阶段一：把 Pinocchio 变成"可生成的东西"（第 1–2 周）

**读**：`skills/robotics-motion-control/references/M01_pinocchio.md`（重点 §2 Model/Data、§4 标量参数化、§5 FK、§7 雅可比）

**核心问题（读不懂就查原文）**
- 为什么 Model 里全是 const、Data 里全是 mutable？这对我们的"const 表进 Flash / 工作区进 DTCM"意味着什么？
- `model.cast<ADCG>()` 为什么不能进循环？
- 末端位姿为什么必须 `updateFramePlacements`，而不是取 `oMi[njoints-1]`？

**做**：写 `host/armgen` 的第一版
```
urdf → pinocchio::Model → ModelTpl<ADCG> → Independent(q) → forwardKinematics → ADFun
     → CodeHandler + LanguageC<float> → 模板渲染 → generated/arm_fk_ee.h
```

**验收脚本**：`host/tests/golden_fk`，随机 1e4 组 q（含限位边界、零位、奇异位形），对比 Pinocchio double：
`max |p_gen − p_ref| < 1e-5 m`、`max |R_gen − R_ref| < 1e-4`

**坑提醒**：末端是 fixed joint 挂上去的 → 生成前先把 `frames[ee].placement` 与最后一个 `jointPlacements` 预乘成常量，否则稳定差一个常量变换。

---

## 2. 阶段二：让生成的代码"能上板"（第 3 周）

**读**：`M06_codegen.md`（全篇）、`M01_pinocchio.md` §8–§11

**核心问题**
- tape 有多大？为什么不能上板？（7-DOF RNEA ≈ 3200 节点 / 100 KB）
- 前向 vs 反向模式怎么选？（`m < n/3` 用反向）
- 生成代码为什么会膨胀？CSE 能省多少？（≈ 30–50%）

**做**
- 按关节把大函数切成 6 段 `static inline`，控制单函数体积
- 建 `-fstack-usage` 报告，把 `.su` 文件纳入 CI
- 写体积预算表：每次生成后自动打印 `arm-none-eabi-size -A -x`

**验收**：`.text ≤ 150 KB`、`最大栈帧 ≤ 2 KB`、`nm` 无 `malloc/free/_sbrk`

**到这一步你就懂了**：为什么 cricket 生成的代码是"平坦语句"而不是函数调用——那是体积与 WCET 的双重考虑。

---

## 3. 阶段三：IK 与动力学（第 4–6 周）

**读**：`M03_ik.md`（§1–§8）、`M01_pinocchio.md` §8–§10

**核心问题**
- 为什么 6 轴非球腕没有闭式解？Pieper 条件是什么？
- DLS 的 λ 为什么必须自适应？float 下 λ 的物理下限是多少？（λ ≥ 1e-3）
- RNEA 两遍递推分别在算什麼？为什么比 CRBA + Cholesky 快？
- 重力补偿 = `rnea(q,0,0)`，那为什么还要单独生成 `gravity`？（省掉零速下的冗余乘法，只多 4.9 KB）

**做**
- `runtime/src/dls.c`：定长 float、6×6 LDLT、λ 自适应、限位 clamp、`{q, residual, iters, status}` 返回
- 生成 `rnea` / `gravity` / `aba`
- 对拍：力矩 1e-3 N·m；IK 在 1000 个可达目标上统计 P99 残差与迭代次数分布

**验收**：P99 ≤ 0.05 mm；奇异位形（伸直的肘）不发散；不收敛时返回上一帧解

---

## 4. 阶段四：上板与实时（第 7–8 周）

**读**：`M11_realtime_cpp.md`、`M12_ros2control.md`

**核心问题**
- 教程里的五条实时铁律哪些在裸机上不适用？替代品是什么？（PREEMPT_RT/SCHED_FIFO 全废，改用链接期禁 malloc + 编译期定尺寸）
- 驱动器状态机为什么必须"未激活时 `cmd = state`"？

**做**
- 1 kHz 定时器中断：采样 → FK → IK → 限幅 → 下发，DWT 打点统计 max/P99
- 上电自检：Flash 里放黄金向量，开机比对，串口打 PASS/FAIL + 最大误差
- 驱动状态机：IDLE → ENABLING → ACTIVE → FAULT，超时/丢帧/位置越界都进 FAULT

**验收**：整周期 max ≤ 300 µs（550k cycles 预算的 55%）；断线后 1 个周期内进入安全态

---

## 5. 阶段五：实机与开源（第 9–12 周）

**读**：`P01_urdf.md`、`M15_minimanip.md`、`M02_dynlibs.md`、`M04_collision.md`（按需）

**做**
- 实机 URDF 接入：重跑全部对拍 + 重新标定惯量（教程 P01 强调 URDF 惯量相对质心）
- 碰撞监控（可选）：胶囊 + 静态 pair 表，单周期 +≤ 10 µs
- 开源化：见 `docs/OPEN-SOURCE.md`

**验收**：实机末端重复定位误差 ≤ 0.1 mm；仓库 CI 绿 + benchmark 表可复现

---

## 6. 可以跳过什么（省 ~40% 时间）

| 章节 | 为什么跳 |
|---|---|
| M07 采样规划 / M08 轨迹优化 / M09 GPU 加速 | 与裸机实时控制零交集 |
| M14 MoveIt2 / MTC | 你只做单臂实时内核，不做产线集成 |
| M05 QP/NLP 工程 | 只在 PC 侧做 MPC 时才需要；我们不做 |
| 足式 10–260（浮动基座、WBC、MPC） | 机械臂是固定基座子集；等做轮足再回看 |
| 30_复合 / 40_仿真 | 后续扩展方向，不是本轮目标 |
| M02 的性能基准部分 | 选型结论已定（Pinocchio 是唯一能 codegen 的），只需看 §9.6 案例 4 |

**只读不做的**：M10 时间参数化（我们让上位机做 Ruckig，MCU 只回放）、M04 的 broadphase/BVH（物体数太少，静态 pair 表更快）。

---

## 7. 怎么证明"学会了"（自测题）

每个模块 3 道，答不上来说明只背了 API：

**Pinocchio**
1. 为什么 `data.M` 只填上三角？直接拿去 Cholesky 会怎样？
2. `LOCAL` / `LOCAL_WORLD_ALIGNED` / `WORLD` 三种雅可比差哪个矩阵？速度 IK 该用哪个？
3. 6 个关节全是 revolute，`nq` 和 `nv` 一定相等吗？什么时候不相等？

**代码生成**
1. 7-DOF RNEA 的 tape 大概多大？为什么不能放到板子上跑？
2. 为什么 AD 算雅可比比手推解析慢？慢多少？
3. `Independent()` 漏掉一个变量，会表现成什么现象？

**IK**
1. 阻尼最小二乘里 λ 太大和太小分别出什么问题？
2. 为什么 float 下 λ=1e-4 等于没有阻尼？
3. 误差定义、雅可比参考系、更新符号，三者不一致会怎样？

**实时**
1. 裸机上"禁止动态分配"怎么变成编译期硬约束？
2. 为什么 WCET 不能用平均值代替？我们的预算怎么算的？
3. 驱动器未使能时为什么要 `cmd = state`？

---

## 8. 时间不够时的 6 周最小路径

```
W1  M01 §2/§5/§7 + 生成 FK/雅可比        → 对拍过
W2  M06 §体积 + 切分 + 栈报告            → 能上板
W3  M03 §3 + DLS 外壳                    → 收敛
W4  M01 §8/§9 + RNEA/重力                 → 力矩对
W5  M11 + 1 kHz 循环 + DWT                → 周期达标
W6  M12 状态机 + 上电自检                 → 实机跑通
```
ABA、碰撞、时间参数化全部延后。
