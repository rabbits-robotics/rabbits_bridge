#include "rabbits_bridge/stm32_bridge_node.hpp"

#include <cerrno>
#include <cstring>
#include <functional>

#include <rabcl/interface/uart.hpp>
#include <rabcl_ros2/utils.hpp>

Stm32Bridge::Stm32Bridge()
: Node("stm32_bridge_node")
{
  device_name_ = "/dev/ttyACM0";
  fd1_ = OpenSerialPort(device_name_);
  if (fd1_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
    rclcpp::shutdown();
  }

  cmd_sub_ = this->create_subscription<rabbits_msgs::msg::AttractsCommand>(
    "cmd", 10,
    std::bind(&Stm32Bridge::CmdCB, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  act_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "act/joint_states", 10);

  read_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&Stm32Bridge::ReadTimerCallback, this));
}

Stm32Bridge::Stm32Bridge(int test_fd)
: Node("stm32_bridge_node"), fd1_(test_fd)
{
  cmd_sub_ = this->create_subscription<rabbits_msgs::msg::AttractsCommand>(
    "cmd", 10,
    std::bind(&Stm32Bridge::CmdCB, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  act_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "act/joint_states", 10);

  read_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&Stm32Bridge::ReadTimerCallback, this));
}

Stm32Bridge::~Stm32Bridge()
{
  if (fd1_ >= 0) {
    close(fd1_);
    fd1_ = -1;
  }
}

int Stm32Bridge::OpenSerialPort(const std::string & device_name)
{
  int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY);

  fcntl(fd1, F_SETFL, 0);
  struct termios conf_tio;
  tcgetattr(fd1, &conf_tio);

  speed_t baudrate = B230400;
  cfsetispeed(&conf_tio, baudrate);
  cfsetospeed(&conf_tio, baudrate);

  conf_tio.c_cflag = CS8 | CLOCAL | CREAD | B230400;
  conf_tio.c_iflag = IGNPAR;
  conf_tio.c_oflag = 0;
  conf_tio.c_lflag = 0;

  // Non-blocking read (VMIN=0, VTIME=0)
  conf_tio.c_cc[VMIN] = 0;
  conf_tio.c_cc[VTIME] = 0;

  tcflush(fd1, TCIOFLUSH);
  tcsetattr(fd1, TCSANOW, &conf_tio);

  return fd1;
}

void Stm32Bridge::CmdCB(const rabbits_msgs::msg::AttractsCommand::SharedPtr msg)
{
  rabcl::Info info;
  rabcl_ros2::Utils::CmdMsgToInfo(*msg, info);

  rabcl::Uart uart;
  uart.PrepareReferencePacket(info);
  SendSerialData(uart.reference_transmit_buffer_);
}

void Stm32Bridge::SendSerialData(const uint8_t * buf)
{
  int rec = write(fd1_, buf, rabcl::Uart::REFERENCE_PACKET_SIZE);
  if (rec < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write: %s", strerror(errno));
  } else if (rec != rabcl::Uart::REFERENCE_PACKET_SIZE) {
    RCLCPP_WARN(this->get_logger(), "Serial Warning: only %d bytes written", rec);
  } else {
    tcdrain(fd1_);
  }
}

void Stm32Bridge::ReadTimerCallback()
{
  while (true) {
    switch (fb_rx_state_) {
      case FeedbackRxState::SCAN_H0: {
          uint8_t b;
          int n = read(fd1_, &b, 1);
          if (n <= 0) {return;}
          if (b == rabcl::Uart::FEEDBACK_HEADER_0) {
            fb_rx_buf_[0] = b;
            fb_rx_state_ = FeedbackRxState::SCAN_H1;
          }
          break;
        }
      case FeedbackRxState::SCAN_H1: {
          uint8_t b;
          int n = read(fd1_, &b, 1);
          if (n <= 0) {return;}
          if (b == rabcl::Uart::FEEDBACK_HEADER_1) {
            fb_rx_buf_[1] = b;
            fb_rx_state_ = FeedbackRxState::READ_REMAIN;
            fb_rx_offset_ = 2;
          } else {
            fb_rx_state_ = (b == rabcl::Uart::FEEDBACK_HEADER_0) ?
              FeedbackRxState::SCAN_H1 : FeedbackRxState::SCAN_H0;
          }
          break;
        }
      case FeedbackRxState::READ_REMAIN: {
          int remain = rabcl::Uart::FEEDBACK_PACKET_SIZE - fb_rx_offset_;
          int n = read(fd1_, fb_rx_buf_ + fb_rx_offset_, remain);
          if (n <= 0) {return;}
          fb_rx_offset_ += n;
          if (fb_rx_offset_ >= rabcl::Uart::FEEDBACK_PACKET_SIZE) {
          // Full packet received — decode and publish
            rabcl::Uart uart;
            std::memcpy(uart.feedback_receive_buffer_, fb_rx_buf_,
            rabcl::Uart::FEEDBACK_PACKET_SIZE);
            rabcl::Info info;
            if (uart.UpdateFeedbackData(info)) {
              auto imu_msg = sensor_msgs::msg::Imu();
              imu_msg.header.stamp = this->now();
              imu_msg.header.frame_id = "imu_link";
              rabcl_ros2::Utils::InfoToImuMsg(info, imu_msg);
              imu_pub_->publish(imu_msg);

              auto js_msg = sensor_msgs::msg::JointState();
              js_msg.header.stamp = this->now();
              rabcl_ros2::Utils::InfoToJointStateMsg(info, js_msg);
              act_joint_state_pub_->publish(js_msg);
            }
            fb_rx_state_ = FeedbackRxState::SCAN_H0;
          }
          break;
        }
    }
  }
}

#ifndef RABBITS_BRIDGE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Stm32Bridge>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
