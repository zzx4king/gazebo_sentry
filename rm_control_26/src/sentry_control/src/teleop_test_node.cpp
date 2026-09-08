// 键盘发布 cmd_vel 测试节点 (teleop_test_node)
// 用途: 手动模拟 Nav2 的 /cmd_vel_smoothed 输出, 配合 control_node 测试 串口->MCU 链路
//
// 键位 (设定目标速度, 节点以 50Hz 持续发布):
//   w : 前进 (linear=+步长)     s : 后退 (linear=-步长)
//   a : 原地左转 (angular=+步长) d : 原地右转 (angular=-步长)
//   q : 线速度 +步长             z : 线速度 -步长
//   e : 角速度 +步长             c : 角速度 -步长
//   x / 空格 : 停止 (全部归零)
//   h : 打印帮助
//
// 用法: ros2 run sentry_control teleoptest_node
// 注意: 测试时勿同时运行 Nav2, 避免双方争抢 /cmd_vel_smoothed

#define _DEFAULT_SOURCE 1

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_control
{

class TeleopTestNode : public rclcpp::Node
{
public:
  explicit TeleopTestNode(const rclcpp::NodeOptions & options)
  : Node("sentry_teleoptest", options)
  {
    // ---------------- 参数 ----------------
    const std::string topic =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_smoothed");
    linear_step_ = declare_parameter<double>("linear_step", 0.2);
    angular_step_ = declare_parameter<double>("angular_step", 0.3);
    max_linear_ = declare_parameter<double>("max_linear", 1.0);
    max_angular_ = declare_parameter<double>("max_angular", 2.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    // 终端切换为无缓冲无回显模式 (退出时自动恢复)
    if (!setup_terminal()) {
      throw std::runtime_error("终端设置失败: 需要在交互式终端 (tty) 中运行");
    }

    RCLCPP_INFO(get_logger(), "发布话题: %s @ 50Hz", topic.c_str());
    print_help();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&TeleopTestNode::loop, this));
  }

  ~TeleopTestNode() override
  {
    restore_terminal();
  }

private:
  static double clamp(double v, double lim)
  {
    if (v > lim) {return lim;}
    if (v < -lim) {return -lim;}
    return v;
  }

  void print_help() const
  {
    ::printf("\n==================== 键盘控制 (速度步长: 线 %.2f m/s / 角 %.2f rad/s) ========="
             "===========\n", linear_step_, angular_step_);
    ::printf("  w : 前进            s : 后退            a : 左转            d : 右转\n");
    ::printf("  q : 线速度 +步长    z : 线速度 -步长    e : 角速度 +步长    c : 角速度 -步长\n");
    ::printf("  x / 空格 : 停止     h : 帮助           Ctrl+C : 退出\n");
    ::printf("当前: linear=%+.2f m/s  angular=%+.2f rad/s\n", linear_, angular_);
    ::printf("==========================================================================="
             "===\n");
    ::fflush(stdout);
  }

  void print_state() const
  {
    ::printf("\r[速度] linear=%+.2f m/s  angular=%+.2f rad/s   ", linear_, angular_);
    ::fflush(stdout);
  }

  bool setup_terminal()
  {
    if (tcgetattr(STDIN_FILENO, &orig_termios_) != 0) {
      return false;
    }
    termios t = orig_termios_;
    t.c_lflag &= ~(ICANON | ECHO);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    return tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0;
  }

  void restore_terminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    ::printf("\n");
  }

  void handle_key(char c)
  {
    bool changed = true;
    switch (std::tolower(static_cast<unsigned char>(c))) {
      case 'w': linear_ = clamp(linear_step_, max_linear_); break;
      case 's': linear_ = clamp(-linear_step_, max_linear_); break;
      case 'a': angular_ = clamp(angular_step_, max_angular_); break;
      case 'd': angular_ = clamp(-angular_step_, max_angular_); break;
      case 'q': linear_ = clamp(linear_ + linear_step_, max_linear_); break;
      case 'z': linear_ = clamp(linear_ - linear_step_, max_linear_); break;
      case 'e': angular_ = clamp(angular_ + angular_step_, max_angular_); break;
      case 'c': angular_ = clamp(angular_ - angular_step_, max_angular_); break;
      case 'x':
      case ' ': linear_ = 0.0; angular_ = 0.0; break;
      case 'h': print_help(); changed = false; break;
      default: changed = false; break;
    }
    if (changed) {
      print_state();
    }
  }

  void loop()
  {
    // 非阻塞读取按键 (Ctrl+C 由终端 ISIG 转 SIGINT 信号, rclcpp 信号处理退出)
    char c = 0;
    while (::read(STDIN_FILENO, &c, 1) == 1) {
      handle_key(c);
    }
    // 50Hz 持续发布当前速度
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_;
    msg.angular.z = angular_;
    pub_->publish(msg);
  }

  // ---------------- 成员 ----------------
  double linear_step_{0.2};
  double angular_step_{0.3};
  double max_linear_{1.0};
  double max_angular_{2.0};
  double linear_{0.0};
  double angular_{0.0};

  termios orig_termios_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sentry_control::TeleopTestNode>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
