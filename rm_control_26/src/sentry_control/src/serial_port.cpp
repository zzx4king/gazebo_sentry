// 串口通信模块: 帧协议打包 + CRC-8/MAXIM + termios 串口读写
// 协议与 /home/robomaster/project/test 目录下已验证的调试脚本 (debug_controller.py 等) 完全一致

#define _DEFAULT_SOURCE 1

#include "sentry_control/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace sentry_control
{

// ==================== CRC-8/MAXIM ====================
// 多项式 0x31 (reversed 0x8C), 初值 0x00, 反射; 校验 "123456789" = 0xA1
uint8_t crc8_maxim(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x01U) {
        crc = static_cast<uint8_t>((crc >> 1) ^ 0x8CU);
      } else {
        crc = static_cast<uint8_t>(crc >> 1);
      }
    }
  }
  return crc;
}

// ==================== 帧打包 ====================
std::vector<uint8_t> pack_frame(uint8_t frame_type, const uint8_t * data, uint8_t len)
{
  std::vector<uint8_t> frame;
  frame.reserve(3U + len + 1U);
  frame.push_back(FRAME_HEADER);
  frame.push_back(frame_type);
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data, data + len);
  }
  frame.push_back(crc8_maxim(frame.data(), frame.size()));
  return frame;
}

std::vector<uint8_t> pack_stop()
{
  return pack_frame(TYPE_STOP, nullptr, 0);
}

std::vector<uint8_t> pack_run()
{
  return pack_frame(TYPE_RUN, nullptr, 0);
}

std::vector<uint8_t> pack_diff(float linear, float angular)
{
  // 小端 f32 x2: 与 Python 端 struct.pack("<ff", linear, angular) 一致
  uint8_t data[8];
  std::memcpy(data, &linear, sizeof(float));
  std::memcpy(data + sizeof(float), &angular, sizeof(float));
  return pack_frame(TYPE_DIFFERENTIAL, data, sizeof(data));
}

// ==================== SerialPort ====================
namespace
{
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B460800;
  }
}
}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & port, int baudrate)
{
  close();
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    close();
    return false;
  }

  const speed_t speed = baud_to_speed(baudrate);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  // 8N1, 无流控, 本地连接使能接收
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cflag &= ~(PARENB | CSTOPB);
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag |= (CLOCAL | CREAD);

  // raw 模式: 关闭行处理/回显/软件流控/输出处理
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR | ISTRIP);
  tio.c_oflag &= ~OPOST;

  // 非阻塞读
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::write(const uint8_t * data, size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

size_t SerialPort::read(uint8_t * buf, size_t len)
{
  if (fd_ < 0 || len == 0) {
    return 0;
  }
  const ssize_t n = ::read(fd_, buf, len);
  return n > 0 ? static_cast<size_t>(n) : 0;
}

}  // namespace sentry_control
