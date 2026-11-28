// 验证：pluginlib 能声明并实例化本控制器
//
// 为什么必须单独测：控制器是 pluginlib 插件，编译通过**不代表**能被
// controller_manager 找到 —— 插件描述 XML 没被 ament 索引登记、类名拼错、
// 导出宏漏写，都会让 `ros2 control load_controller` 在运行时才报错。
// 这个测试把"能不能加载"提前到构建期。
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

namespace {
constexpr char kClass[] = "hw_arm_controllers/CartesianImpedanceController";
constexpr char kBase[] = "controller_interface::ControllerInterface";
}  // namespace

TEST(CartesianImpedanceControllerPlugin, IsDeclared) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", kBase);
  const auto names = loader.getDeclaredClasses();
  ASSERT_FALSE(names.empty());
  EXPECT_NE(std::find(names.begin(), names.end(), kClass), names.end())
      << "pluginlib 没声明 " << kClass;
}

TEST(CartesianImpedanceControllerPlugin, CanBeInstantiated) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
      "controller_interface", kBase);
  std::shared_ptr<controller_interface::ControllerInterface> inst;
  ASSERT_NO_THROW(inst = loader.createSharedInstance(kClass));
  ASSERT_NE(inst, nullptr);
}
