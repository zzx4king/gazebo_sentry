// sentry_control 节点
// 功能: 订阅 Nav2 输出的 cmd_vel (geometry_msgs/Twist), 提取 linear.x 与 angular.z,
//       将角速度取反后除以缩放系数(默认 1.42), 按串口协议 (Header+Type+Len+Data+CRC8)
//       以固定频率持续发送差速指令至 MCU。
//
// 串口协议与 /home/robomaster/project/test 目录下已验证脚本一致:
//   启动流程: stop(安全起步) -> run(进入运行状态)
//   差速指令: Type 0x10, linear(f32 LE) + angular(f32 LE)
//   持续发送间隔须 < MCU 端 500ms 超时 (默认 50Hz)
//   退出时自动发送 stop 帧安全停车
//
// 安全保护: cmd_vel 超时(默认 0.5s)未更新时自动发送零速, 防止 Nav2 异常退出时失控

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_control/serial_port.hpp"

namespace sentry_control
{

class ControlNode : public rclcpp::Node
{
public:
  explicit ControlNode(const rclcpp::NodeOptions & options)
  : Node("sentry_control", options)
  {
    // ---------------- 参数 ----------------
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 460800);
    angular_divisor_ = declare_parameter<double>("angular_divisor", 1.42);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    const double send_rate = declare_parameter<double>("send_rate", 50.0);
    const std::string topic =
      declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

    if (angular_divisor_ == 0.0) {
      throw std::invalid_argument("angular_divisor 不能为 0");
    }

    // ---------------- 打开串口 ----------------
    if (!serial_.open(port_, baudrate_)) {
      RCLCPP_FATAL(get_logger(), "打开串口失败: %s @ %d (检查设备存在/权限/是否被占用)",
        port_.c_str(), baudrate_);
      throw std::runtime_error("serial open failed: " + port_);
    }
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d", port_.c_str(), baudrate_);

    // ---------------- 安全起步: stop -> run ----------------
    write_frame(pack_stop(), "stop (安全起步)");
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    write_frame(pack_run(), "run (进入运行状态)");

    // ---------------- 订阅 cmd_vel ----------------
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
        last_cmd_time_ = now();
        cmd_received_ = true;
      });
    RCLCPP_INFO(get_logger(), "已订阅话题: %s", topic.c_str());

    // ---------------- 定时发送 (默认 50Hz, 须 < MCU 500ms 超时) ----------------
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / send_rate)),
      std::bind(&ControlNode::send_command, this));

    RCLCPP_INFO(get_logger(),
      "sentry_control 启动完成: angular /= %.3f, 发送频率 %.0fHz, cmd_vel 超时 %.2fs",
      angular_divisor_, send_rate, cmd_vel_timeout_);
  }

  ~ControlNode() override
  {
    // 退出前发送 stop 帧安全停止底盘 (与已验证脚本行为一致)
    try {
      if (serial_.is_open()) {
        write_frame(pack_stop(), "stop (退出安全停止)");
      }
      serial_.close();
    } catch (...) {
    }
  }

private:
  void write_frame(const std::vector<uint8_t> & frame, const char * desc)
  {
    if (!serial_.write(frame.data(), frame.size())) {
      RCLCPP_ERROR(get_logger(), "串口写入失败: %s", desc);
      return;
    }
    RCLCPP_DEBUG(get_logger(), "[发送] %-18s %s", desc, hex_str(frame).c_str());
  }

  void send_command()
  {
    double linear = 0.0;
    double angular = 0.0;

    if (cmd_received_ && (now() - last_cmd_time_).seconds() <= cmd_vel_timeout_) {
      linear = cmd_.linear.x;
      angular = cmd_.angular.z;
    }
    // 超时/未收到指令时保持零速 (安全保护)

    // 角速度先取反 (实车转向与 ROS 约定相反: ROS 逆时针为正, MCU 顺时针为正),
    // 再除以缩放系数 (默认 1.42) 后发送
    angular = -angular;
    angular /= angular_divisor_;

    // 过滤极小值, 避免浮点噪声造成底盘微动
    constexpr double EPS = 1e-4;
    if (std::fabs(linear) < EPS) {linear = 0.0;}
    if (std::fabs(angular) < EPS) {angular = 0.0;}

    write_frame(
      pack_diff(static_cast<float>(linear), static_cast<float>(angular)),
      "diff 差速指令");
  }

  static std::string hex_str(const std::vector<uint8_t> & frame)
  {
    std::string s;
    char buf[4];
    for (const auto b : frame) {
      std::snprintf(buf, sizeof(buf), "%02X ", b);
      s += buf;
    }
    return s;
  }

  // ---------------- 成员 ----------------
  std::string port_;
  int baudrate_{460800};
  double angular_divisor_{1.42};
  double cmd_vel_timeout_{0.5};

  SerialPort serial_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_cmd_time_;
  bool cmd_received_{false};
};

}  // namespace sentry_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_control::ControlNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
