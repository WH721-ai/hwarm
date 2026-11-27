# 机械臂算法库（Pinocchio + CppADCodeGen → STM32H723 / 达妙 DM-MC-Board02）总体方案

> 版本 **v0.4**（板卡 DM-MC-Board02 / STM32H723VGT6；**烧录路径 A 定案**；硬件齐备）
> 工作空间 `/home/user123/Cricket` · 参考工程 `/home/user123/cricket`（Cricket，已装好并验证）

---

## 0. 结论先读（TL;DR）

1. **架构不变**：Pinocchio / CppAD / CppADCodeGen 只在宿主 PC 上跑，**离线**把运动学 + 动力学编译成平坦 C；目标端是零依赖、零堆、固定栈的 C99 库。
2. **体积实测（Panda 7 轴，含惯量，本轮新测）**：FK 4.7 KB + 雅可比 6.6 KB + **RNEA 14.0 KB** + **重力项 4.9 KB** + **ABA 39.7 KB** ≈ **70 KB 源码**，编译后约 60–100 KB Flash。1 MB Flash 用不到 10%，**非常宽裕**。
3. **真正的约束是栈**：ABA 一次要 **260 个 float 临时变量 ≈ 1.04 KB**，裸机默认栈（常 1–2 KB）会直接溢出 → 链接脚本把栈设到 **16 KB**。
4. **0.1 mm 精度 `float` 足够**（误差预算 2–5 µm，余量 20–50 倍）。
5. **烧录 ≠ 串口**。你这块板子只有 **SWD 排针（PA13/PA14）**，没有板载调试器 → 必须外接 **ST-Link / J-Link / DAP-Link**。串口只传数据。
6. 唯一硬阻塞：本机没有 `arm-none-eabi-gcc`（已确认可从 developer.arm.com 拉，580 KB/s）。

---

## 1. 决策表（已确认）

| 项 | 决策 | 影响 |
|---|---|---|
| 板卡 | **达妙 DM-MC-Board02** | 见 §2 |
| MCU | **STM32H723VGT6**（Cortex-M7 @550 MHz，1 MB Flash，LQFP100） | 1 MB Flash 够用；堆栈见 §4.3 |
| 算法 | 运动学 + **RNEA + 重力补偿 + ABA** | 三个动力学函数都生成，§5 |
| 精度 | 0.1 mm | `float`，§4.2 |
| 运行环境 | 裸机 | 链接脚本 / 启动文件自己管 |
| 工具链 | 双轨：CMake+GCC（CI/验证）+ Keil armclang（你现有环境） | §7 |
| 烧录 | **路径 A**：Linux/CI 出 `.hex` → Windows 烧，零系统改动 | §3.1 |
| 部署形态 | PC 与 MCU 共用同一份生成代码；先 PC 后下沉（见 `ARCHITECTURE-OPTIONS.md`） | P1→P3 |
| URDF | 后续给标准实机 URDF | 生成器与机型解耦，§6 |

---

## 2. 板卡：DM-MC-Board02

来源：[达妙 DM-MC02 资料仓](https://gitee.com/kit-miao/dm-mc02)（使用说明书 V1.1）

| 项目 | 内容 |
|---|---|
| 主控 | STM32H723VGT6（Cortex-M7，550 MHz，1 MB Flash，LQFP100） |
| 供电 | 12–24 V（支持 6S），防反接 + 缓启动 |
| 执行器接口 | **3× CANFD**、2× RS485（10 Mbps）、4× PWM（5 V@2A 可控电源）、SBUS |
| 通信/调试 | **1× USB 全速（device）**、2× TTL 串口（一路可对外供 5 V）、**SWD（PA13/PA14）** |
| 板载外设 | BMI088 IMU、WS2812 彩灯、蜂鸣器、按键、W25Q64 QSPI Flash、LCD 扩展、摄像头接口 |
| 官方例程 | **全部是 Keil MDK 工程**（`CtrBoard-H7_*`） |

**内存地图**（[Zephyr H723 dtsi](https://github.com/zephyrproject-rtos/zephyr/blob/main/dts/arm/st/h7/stm32h723.dtsi) 核对）

| 区域 | 地址 | 大小 | 用途 |
|---|---|---|---|
| Flash | `0x0800_0000` | 1 MB（128 KB 扇区） | 代码 + 常量表 |
| ITCM | `0x0000_0000` | 64 KB | 可选热点（`arm_ik` 内层） |
| DTCM | `0x2000_0000` | 128 KB | **栈** + `.data/.bss`（快，DMA 不可访问） |
| AXI SRAM | `0x2400_0000` | 320 KB | DMA 缓冲（UART/CAN 收发） |
| SRAM1/2/4 | `0x3000_0000` / `0x3000_4000` / `0x3800_0000` | 各 16 KB | 备用 |

---

## 3. 烧录与连线（**回答"是不是串口连接"**）

**不是。这块板子上有三条互不相干的链路：**

```
        ┌──────────────── 12–24V 电源 ────────────────┐
        │                                              │
  [Linux PC] ──USB──> [ST-Link / J-Link] ──SWD 4pin──> MCU   ← ① 烧录/调试（必须）
  [Linux PC] ──USB-TTL 模块──> 板载 TTL 串口 ─────────> MCU   ← ② 运行数据/日志（可选）
  [Linux PC] ──USB 线──> 板载 USB FS 口（CDC 虚拟串口）─> MCU  ← ②' 同上，一根线搞定（可选）
```

| 链路 | 接口 | 用途 | 硬件 |
|---|---|---|---|
| ① 烧录/调试 | **SWD**：PA13(SWDIO)、PA14(SWCLK)、3.3V、GND | 下载程序、单步、读变量 | ✅ 已有 ST-Link |
| ② 数据 | 2× TTL 串口 | printf 日志、上位机通信 | ✅ 已有 USB-TTL |
| ②' 数据 | 板载 USB FS（做 CDC-ACM） | 同上，一根线搞定 | ✅ 板载 |
| 电源 | 12–24 V | 供电 | ✅ 已有 |

### 3.1 ARM 工具链是干什么的

**一句话：它把 `.c` 翻译成 H723 能执行的机器码。** 你现在 Linux 上的 `g++` 只会产出 x86-64 指令，H723 一条都执行不了。

一套 `arm-none-eabi-gcc` 里包含：

| 工具 | 作用 | 我们什么时候用 |
|---|---|---|
| `arm-none-eabi-gcc` | C 编译器 | 编译 `runtime/src/*.c` |
| `arm-none-eabi-g++` | C++ 编译器 | 基本不用（目标端是纯 C） |
| `as` / `ld` | 汇编 / 链接（配 linker script） | 生成 `.elf` |
| `objcopy` | `.elf` → **`.hex` / `.bin`** | **交给 Windows 烧录的就是它** |
| `size` | 统计 Flash/RAM 占用 | CI 里卡体积预算 |
| `nm` / `objdump` | 查符号、反汇编 | 查有没有偷偷 `malloc`、看热点汇编 |
| `gdb` + `openocd` | 在线调试 | 可选（你走 Keil，不需要） |

**但注意：走路径 A 的话，工具链不是"必须"的。** 两条子路线：

| | 谁编译 | 需要装 GCC 工具链吗 | 优点 | 缺点 |
|---|---|---|---|---|
| **A1（推荐）** | Linux 上 GCC 编，出 `.hex` | ✅ 需要（140 MB） | CI 自动化、体积/栈/周期全部可脚本化、我能在 Linux 侧先验证 | 装一次 |
| A2 | Windows Keil（armclang）编 | ❌ 不需要 | 零安装，跟你现有习惯一致 | 我只能给源码，无法自动跑 CI/体积统计 |

我的建议是**仍然装 A1**，理由：我们的验收项 L1（对拍）、L3（体积/栈）、L4（周期）全都依赖自动化编译；而且生成的代码是纯 C99，A1/A2 两条路都能编，装了不亏。**工具链只装进 `Cricket/toolchain/`，不碰系统。**

### 3.2 串口是干什么的

**一句话：它是裸机上唯一的"printf"。** 板子没有屏幕，出问题时你只能靠它说话。三个用途，按重要性排：

1. **调试日志（最重要）**：`printf("q=%f err=%f\n", ...)` → USB-TTL → PC 串口助手/`minicom`。查"为什么 IK 不收敛""为什么这组 q 算出来是 NaN"基本全靠它。
2. **运行数据 / 上位机通信**：把关节角、力矩、单周期耗时打包发回 PC，用 Python 实时画图；反过来 PC 下发目标位姿给板子做台架测试。
3. **自检报告**：L5 硬件在环——上电跑黄金向量，串口打印 `PASS/FAIL + 最大误差`。

注意三件事：
- **串口不能烧录，也不能单步**。烧录/单步只走 SWD + ST-Link + Keil。
- 板子有 **2 路 TTL 串口**（需 USB-TTL 模块）和 **1 个 USB FS 口**。用 UART 最省事（几行寄存器）；用 USB CDC 少一根线，但要带 USB 协议栈（CubeMX 生成 / TinyUSB，约 10–20 KB Flash）。**建议先用 UART**，跑通再说。
- 串口只是"辅助观测"，不进算法库。算法库本身零 IO、零依赖。

### 3.3 路径 A 的完整流程（已定案）

```
Linux:  cmake --build build-arm        →  build-arm/arm_kin.hex
        （objcopy 从 .elf 生成，CI 里自动跑）
   │
   │  VMware 共享文件夹（/mnt/hgfs/... 或 Windows 共享目录）
   ▼
Windows: STM32CubeProgrammer 拖入 .hex → 烧录     ← 最省事
         或 Keil 打开工程 → Download              ← 要调试时用这个
```

要点：
- **只烧 `.hex` 用 STM32CubeProgrammer 更简单**（不用建工程，拖进去点 Start）；需要**单步调试**时才用 Keil 建工程。
- 烧录器接线：ST-Link 的 SWDIO→PA13、SWCLK→PA14、GND→GND、3.3V 可接可不接（板子自己供电）。
- 板子供电 12–24 V，**先上电再连 ST-Link**（或按 ST-Link 手册的复位顺序）。

### 3.4 备选：Linux 直接烧（目前不用）

| 方案 | 需要的硬件 | 系统改动 | 状态 |
|---|---|---|---|
| B. ST-Link + pyocd | ST-Link | ⛔ 一条 udev 规则 | 暂不需要 |
| C. J-Link + JLinkExe | J-Link | ⛔ 一条 udev 规则 | 暂不需要 |
| D. USB DFU（BOOT0） | 仅 USB 线 | ⛔ udev 规则 + 改 BOOT0 | 不推荐 |

真要用时只需一个文件：`/etc/udev/rules.d/49-stlink.rules`（`SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666", GROUP="plugdev"`）+ `udevadm control --reload-rules`。**这是系统改动，我会先停下来问你。**

---

## 4. 目标端运行时设计

### 4.1 API（纯 C99，无 GNU 扩展 → Keil armclang 可直接编）

```c
/* arm_kin.h —— 维度由生成器写入 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define ARM_NQ   6
#define ARM_NV   6

/* ---------- 运动学 ---------- */
void arm_fk_ee   (const float q[ARM_NQ], float T[12]);   /* 位置3 + 旋转9(列主序) */
void arm_jacobian(const float q[ARM_NQ], float J[6*ARM_NV]);

/* ---------- 逆运动学 ---------- */
typedef struct {
    uint32_t max_iter;
    float    tol_pos;   /* [m]   建议 1e-5 (=0.01 mm) */
    float    tol_rot;   /* [rad] 建议 1e-4 */
    float    lambda;    /* DLS 阻尼，0 = 纯牛顿 */
} arm_ik_opts;

bool arm_ik(const float T[12], const float q_init[ARM_NQ],
            float q_out[ARM_NQ], const arm_ik_opts *opts);

/* ---------- 动力学 ---------- */
void arm_rnea   (const float q[], const float dq[], const float ddq[], float tau[]);
void arm_gravity(const float q[], float g[]);
void arm_aba    (const float q[], const float dq[], const float tau[], float ddq[]);

/* ---------- 工具（手写）---------- */
void arm_interp (const float a[], const float b[], float t, float out[]);
void arm_clamp  (float q[]);
```

### 4.2 精度预算（0.1 mm）

| 误差源 | 量级 |
|---|---|
| float 机器精度 | 1.19e-7 相对 |
| 6 轴串联累积（臂展 ~1 m） | ≈ 0.7 µm |
| `sinf/cosf` 库误差 | 1–2 ulp |
| **合计位置误差** | **≈ 2–5 µm**（比 0.1 mm 小 20–50 倍） |

→ 全链路 `float`；IK 收敛判据取 `tol_pos = 1e-5 m`（0.01 mm），比目标再紧 10 倍留调节余量。

### 4.3 栈与内存（**H723 上最该盯的地方**）

| 函数 | 生成行数 | 临时变量 | 栈占用 |
|---|---|---|---|
| `arm_fk_ee` | 99 | 39 float | ~200 B |
| `arm_jacobian` | 144 | 39 float | ~250 B |
| `arm_rnea` | 215 | 129 float | ~600 B |
| `arm_gravity` | 98 | 55 float | ~300 B |
| `arm_aba` | 617 | **260 float** | **~1.1 KB** |
| `arm_ik`（8 次迭代，调 FK/Jac） | — | 复用 | ~3–5 KB |

→ **链接脚本栈 16 KB**；开 `-fstack-usage` 把 `.su` 文件纳入 CI。
→ `.bss/.data` 应为 0（全部栈 + 常量表进 Flash）。
→ DMA 缓冲必须显式放 AXI SRAM。

### 4.4 编译选项

```cmake
-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
-O3 -ffast-math -fno-common -fno-exceptions -fno-rtti -fno-threadsafe-statics
-ffunction-sections -fdata-sections -fno-unwind-tables -fno-asynchronous-unwind-tables
-Wall -Wextra -Wconversion -Wdouble-promotion
--specs=nano.specs
-Wl,--gc-sections
```

Keil（armclang）等价：`-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -O3 -ffast-math --c99`。

---

## 5. 算法路线与体积（本轮实测）

用本机已装好的 Pinocchio + CppAD + CppADCodeGen，对 Panda（7 轴、带惯量）实测：

| 生成函数 | 源码字符 | 行数 | 临时变量 | 输出 | 说明 |
|---|---|---|---|---|---|
| `eefk`（末端位姿） | 4,673 | 99 | 39 | 12 | 位置 + 旋转矩阵 |
| `eejac`（位姿 + 6×7 雅可比） | 6,598 | 144 | 39 | 54 | |
| **`rnea`** | **14,039** | **215** | **129** | 7 | (q,dq,ddq) → τ |
| **`gravity`** | **4,887** | **98** | **55** | 7 | q → τ_g |
| **`aba`** | **39,740** | **617** | **260** | 7 | (q,dq,τ) → ddq |
| 合计 | **~70 KB** | | | | 6 轴实机会更小 |

**结论**：Flash 完全不是问题；**周期**和**栈**才是要标定的。

### 5.1 三个动力学函数怎么用

| 函数 | Pinocchio 离线追踪 | 典型用途 |
|---|---|---|
| `arm_rnea` | `pinocchio::rnea(model, data, q, v, a)` | 计算力矩控制、逆动力学校核 |
| `arm_gravity` | `pinocchio::computeGeneralizedGravity(model, data, q)` | 重力补偿（每周期都跑，最常用） |
| `arm_aba` | `pinocchio::aba(model, data, q, v, tau)` | 前向动力学、状态预测、仿真 |

> `arm_gravity` 虽然可以用 `arm_rnea(q,0,0)` 代替，但**单独生成只多 4.9 KB、运行更快**（省掉零速下的冗余乘法），所以三个都生成。

### 5.2 IK（0.1 mm）

```
q ← q_init
for iter in 0..max_iter:
    T_cur, J ← 一次调用同时出位姿和雅可比
    e ← [p_t - p_cur ; 0.5·vee(R_curᵀR_t - R_tᵀR_cur)]
    if |e_p| < tol_pos and |e_r| < tol_rot: return true
    Δq ← (JᵀJ + λ²I)⁻¹ Jᵀe        # 6×6 手写 Cholesky
    q  ← clamp(q ⊕ Δq)
return false
```
- 不展开迭代（展开版 +40 KB，只在周期抖动不可接受时再上）；
- 奇异位形用自适应阻尼 `λ`。

---

## 6. 工程结构（URDF 后补友好）

```
Cricket/
├── CMakeLists.txt
├── cmake/{toolchain-arm-none-eabi.cmake, deps.cmake}
├── config/{robot_placeholder.yaml, robot_v1.yaml}
├── models/{placeholder/robot.urdf, v1/robot.urdf}
├── host/{armgen/, tests/}
├── runtime/{include/arm/, src/}
├── generated/                 # *.h + meta.json（urdf_sha256、armgen_version）
├── board/{startup, linker, uart, dwt}   # 裸机示例
├── templates/
├── third_party/inja/
├── toolchain/                 # 本地 arm-none-eabi（不入库）
├── skills/                    # 教程知识包（本项目学习用，见 README）
│   └── robotics-motion-control/
└── docs/                      # PLAN / LEARNING-PATH / OPEN-SOURCE
```

**与机型解耦**
1. `armgen` 只吃 `--urdf + --config`，代码里零机器人常量；
2. 生成物写 `urdf_sha256` + `armgen_version`，换 URDF 一眼看出过期；
3. 对拍 golden 向量按「机器人名 + 哈希」存，CI 强制重跑。

---

## 7. 双工具链策略（Keil + GCC）

| 用途 | 工具链 | 说明 |
|---|---|---|
| CI / 宿主对拍 / 体积统计 | **CMake + arm-none-eabi-gcc 13.3** | 可脚本化，回归用 |
| 板卡烧录 / 单步调试 | **Keil MDK + ST-Link**（你现有） | 达妙官方例程就是 Keil 工程，直接沿用 |
| 代码本体 | **纯 C99，无 GNU 扩展** | 两边都能编 |

交付物两份：`libarm_kin.a`（GCC）+ **源码目录**（Keil 里直接加进工程）。
Keil 侧只需要：把 `runtime/src/*.c` + `generated/*.h` 加入工程，设 `--c99`，栈改 16 KB。

---

## 8. 验证与验收

| 层级 | 方法 | 通过标准 |
|---|---|---|
| L1 生成正确性 | 宿主随机 1e6 组 q，对比 Pinocchio | FK 位置 < **1e-5 m**；旋转 < 1e-4；力矩 < 1e-3 N·m |
| L2 目标端一致性 | QEMU `mps2-an500`（Cortex-M7）+ 半主机 | 与 L1 同组黄金向量一致 |
| L3 资源 | `arm-none-eabi-size -A -x`、`nm` 查堆符号 | Flash ≤ 150 KB；`.bss` = 0；无 `malloc` |
| L4 周期 | DWT->CYCCNT，1000 次取 max/mean | FK ≤ 3 µs；RNEA ≤ 6 µs；ABA ≤ 10 µs；IK ≤ 40 µs @550 MHz |
| **L5 硬件在环** | 黄金向量进 Flash，上电自检 + 串口打印 | 全 PASS，**实测位置误差 ≤ 0.05 mm** |
| L6 可复现 | CI 重新生成 → `git diff --exit-code generated/` | 无差异 |

---

## 9. 里程碑

| 阶段 | 内容 | 产出 | 工期 |
|---|---|---|---|
| **M0** ✅ | 工具链入 workspace + 过渡 URDF 展开入库 + 板卡链路 demo（USART1 打印固件已编出） | `toolchain/`、`models/placeholder/robot.urdf`、`board/build-out/arm_board_demo.hex` | **已完成**（待上板确认串口输出） |
| **M1** | `armgen` 打样：`fk_ee` / `jacobian` 生成 + L1 对拍 | `host/armgen`、`generated/` | 2 d |
| **M2** | `arm_kin` + 交叉编译 + L3/L4 基线 + 链接脚本（栈 16 KB） | `runtime/`、size/cycle 报告 | 2 d |
| **M3** | `arm_ik`（DLS + 6×6 Cholesky） | `ik.c` | 2–3 d |
| **M4** | 动力学三件套 `rnea/gravity/aba` 生成 + 栈预算实测 | `dyn.c` | 2 d |
| **M5** | 板卡联调（CAN 接电机 / 串口上位机）+ 文档 + CI | `board/`、README | 2 d |

**合计 ≈ 11–13 个工作日。**

---

## 10. 风险与对策

| # | 风险 | 影响 | 对策 |
|---|---|---|---|
| R1 | 无 ARM 工具链 | 无法交叉编译 | 下 `arm-gnu-toolchain-13.3.rel1` 到 `toolchain/`（~140 MB，580 KB/s） |
| R2 | **裸机默认栈 1–2 KB，ABA 要 1.1 KB** | 随机 HardFault，极难查 | 栈 16 KB + `-fstack-usage` 进 CI |
| R3 | **板子无板载调试器** | 没有 ST-Link/J-Link 就烧不了 | 确认你是否有调试器；没有建议买一个 ST-Link V3（几十块） |
| R4 | VMware USB 直通 | Linux 看不到 ST-Link | 方案 A 不依赖；要用 B 先在 VMware 里直通 |
| R5 | 过渡 URDF 惯性是建模值 | 动力学系数不准 | 实机 URDF 到位后重生成 + 称重校核 |
| R6 | URDF 换成实机后结构变化 | 维度/体积变化 | 生成器完全 URDF 驱动 + 哈希标记 + CI 重跑 |
| R7 | `-ffast-math` 破坏 NaN/Inf | IK 发散静默出错 | 迭代后显式 `isfinite` + 关节范围检查 |
| R8 | CppADCodeGen 版本漂移 | 构建失败 | pin `v2.5.0` + patch 入库 |
| R9 | Keil/GCC 双工具链差异 | 编译告警/行为差异 | 纯 C99 + `-pedantic` 双端验证 |
| R10 | 8 GB 内存 OOM | 编译被杀 | 生成器 `-j2`，目标端 `-j4` |

---

## 11. 还需要你确认

已定：硬件齐备、烧录走 **路径 A**、动力学三件套。

剩下这些（都不阻塞我开工）：

1. **工具链要不要装**（A1 推荐装 / A2 完全不装，用 Keil 编）。
2. **串口用哪一路**：说明书那页是图，我拿不到具体 USART 编号——你给我管脚图，或直接说"用 USART3 那路"。也可以先用板载 USB 做 CDC。
3. **实机 URDF 大概什么时候给**（决定 M4 之后是否要重跑一轮）。

---

## 附录 A：本轮实测命令与数据

环境：本机 Pinocchio 4.1.0 + CppAD 2024 + CppADCodeGen v2.5.0，模型 `panda.urdf`（7 轴，含惯量）。

```
nq=7 nv=7
RNEA   (q,v,a)->tau : chars=14039 lines=215 vars=129 outs=7
GRAVITY(q)->tau     : chars= 4887 lines= 98 vars= 55 outs=7
```
（ABA 与 FK/雅可比由 `fkcc_gen` 实测：39,740 / 4,673 / 6,598 字符）

## 附录 B：本方案对系统的影响面

- **写入**：仅 `/home/user123/Cricket/**`（本轮只更新了 `docs/PLAN.md`，测量产物在 `.scratch/` 内）
- **读取**：cricket 安装目录、`/opt/ros/jazzy`、`/usr/local`、达妙资料仓
- **可能需要的系统改动**（会先停下问你）：仅 §3 方案 B/C 的那一条 udev 规则
