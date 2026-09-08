#ifndef SENTRY_CONTROL__SERIAL_PORT_HPP_
#define SENTRY_CONTROL__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sentry_control
{

// ==================== 协议常量 (与 MCU 端 serial_cmd.c / test 目录调试脚本一致) ====================
// 帧格式: Header(0xA5) + Type + Len + Data[Len] + CRC8/MAXIM
constexpr uint8_t FRAME_HEADER = 0xA5;
constexpr uint8_t TYPE_STOP = 0x00;          // 状态帧 (Len=0): 切换 STOP, 底盘停转
constexpr uint8_t TYPE_RUN = 0x01;           // 状态帧 (Len=0): 切换 RUN, 允许驱动
constexpr uint8_t TYPE_DIFFERENTIAL = 0x10;  // 差速指令 (Len=8): linear(f32 LE) + angular(f32 LE)

// CRC-8/MAXIM: 多项式 0x31 (reversed 0x8C), 初值 0x00, 输入/输出反射
// 校验 "123456789" = 0xA1 (与 MCU 端 serial_cmd.c 一致)
uint8_t crc8_maxim(const uint8_t * data, size_t len);

// 打包一帧: Header + Type + Len + Data + CRC
std::vector<uint8_t> pack_frame(uint8_t frame_type, const uint8_t * data, uint8_t len);

// 状态帧打包
std::vector<uint8_t> pack_stop();
std::vector<uint8_t> pack_run();

// 差速指令打包: linear(f32 小端) + angular(f32 小端) 共 8 字节
// 坐标系: x 向前, yaw 逆时针为正; linear 单位 m/s, angular 单位 rad/s
std::vector<uint8_t> pack_diff(float linear, float angular);

// ==================== 串口封装 (Linux termios, 8N1, 无流控) ====================
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // 打开串口并配置为 raw 模式, 失败返回 false
  bool open(const std::string & port, int baudrate);
  void close();
  bool is_open() const;

  // 写入 len 字节, 全部写出返回 true
  bool write(const uint8_t * data, size_t len);

  // 非阻塞读取 (用于读取 MCU 调试回传), 返回实际读取字节数
  size_t read(uint8_t * buf, size_t len);

private:
  int fd_ = -1;
};

}  // namespace sentry_control

#endif  // SENTRY_CONTROL__SERIAL_PORT_HPP_
