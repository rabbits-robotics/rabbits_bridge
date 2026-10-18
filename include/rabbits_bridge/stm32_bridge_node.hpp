#ifndef RABBITS_BRIDGE__STM32_BRIDGE_NODE_HPP_
#define RABBITS_BRIDGE__STM32_BRIDGE_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <rabbits_msgs/msg/attracts_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rabcl/interface/uart.hpp>

class Stm32Bridge : public rclcpp::Node
{
public:
  Stm32Bridge();
  explicit Stm32Bridge(int test_fd);
  ~Stm32Bridge();

private:
  int OpenSerialPort(const std::string & device_name);
  void CmdCB(const rabbits_msgs::msg::AttractsCommand::SharedPtr msg);
  void SendSerialData(const uint8_t * buf);
  void ReadTimerCallback();

  rclcpp::Subscription<rabbits_msgs::msg::AttractsCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr act_joint_state_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  std::string device_name_;
  int fd1_ = -1;

  // Feedback receive state machine
  enum class FeedbackRxState { SCAN_H0, SCAN_H1, READ_REMAIN };
  FeedbackRxState fb_rx_state_ = FeedbackRxState::SCAN_H0;
  uint8_t fb_rx_buf_[rabcl::Uart::FEEDBACK_PACKET_SIZE];
  int fb_rx_offset_ = 0;
};

#endif  // RABBITS_BRIDGE__STM32_BRIDGE_NODE_HPP_
