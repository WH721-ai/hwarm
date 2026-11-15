# openarm_can：拉取、编译与用法

> 源码：`https://github.com/enactic/openarm_can`（Apache-2.0，v1.2.9）
> 位置：`~/hw_arm/hw_arm_ws/src/openarm_can`

## 1. 编译

```bash
cd ~/hw_arm/hw_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openarm_can --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 约 **10 秒**完成（只有 `-Wunused-parameter` / `system()` 返回值未检查之类的 warning）
- 依赖：`libcli11-dev`（本机已装 2.4.1）、Linux SocketCAN（glibc 自带）
- `package.xml` 里 `build_type = cmake`（**不是** ament_cmake），colcon 用 cmake 扩展构建

## 2. 产物

```
install/openarm_can/
├── lib/libopenarm_can.so.1.4.1         # 主库
├── lib/cmake/OpenArmCAN/               # find_package(OpenArmCAN) 的配置
├── include/openarm/
│   ├── canbus/{can_socket,can_device,can_device_collection}.hpp
│   ├── damiao_motor/{dm_motor,dm_motor_control,dm_motor_device,...}.hpp
│   └── can/socket/{openarm,arm_component,gripper_component}.hpp
└── bin/
    ├── openarm-can-cli                  # 命令行工具（配置/使能/监控/标定）
    ├── openarm-can-health               # 健康检查
    ├── openarm-can-demo                 # 最小示例
    ├── openarm-can-motor-sampling-check # 采样率检查
    ├── openarm-can-zero-position-calibration
    └── openarm-can-configure-socketcan-4-arms
```

## 3. 在别的工程里用

```cmake
find_package(OpenArmCAN REQUIRED)
target_link_libraries(my_node PRIVATE OpenArmCAN::openarm_can)   # ← 注意命名空间
```

> 坑：**不是** `openarm_can`，是 **`OpenArmCAN::openarm_can`**。写成前者不会报错，但 include 路径不会传递，编译时 `<openarm/...>` 找不到。

头文件用法：

```cpp
#include <openarm/can/socket/openarm.hpp>
openarm::can::socket::OpenArm arm("can0", /*enable_fd=*/true);   // 构造时就会打开 SocketCAN
```

**运行前必须有 can0**，否则构造函数直接抛 `openarm::canbus::CANSocketException`：

```
Socket error: Failed to initialize socket for interface: can0
```

## 4. CLI 能力（`openarm-can-cli --help`）

| 分组 | 子命令 |
|---|---|
| 网络/硬件 | `can_configure`（配 1M/5M FD）、`discover`（扫 1M/5M/8M/10M 找电机） |
| 电机配置 | `change_id`、`change_baud`、`show_param`、`write_param`、`set_zero` |
| 运行调试 | `enable` / `disable`（力矩开关）、`clear_error`、`monitor`（实时 pos/vel/torque/temp）、`diagnose` |

**这基本覆盖了"接一台新电机"要做的全部动作**：配总线 → 扫描 → 改 ID → 置零 → 使能 → 监控。

## 5. 对我们项目的价值

- 直接当 **ros2_control 硬件插件**的 CAN 层：不用自己写帧打包、状态机、CANFD 参数。
- `arm_component` / `gripper_component` 已经把"机械臂 6-7 个电机 + 夹爪"的语义封装好了。
- CLI 的 `monitor` / `diagnose` 可以直接当**上机调试工具**用。

## 6. 注意

- 需要 `can0` 且通常要 `sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on`（CLI 里有 `can_configure` 子命令代劳）。
- 无硬件时只能验证到"编译 + 链接 + 构造抛异常"这一层，真正的收发要等适配器接上。
