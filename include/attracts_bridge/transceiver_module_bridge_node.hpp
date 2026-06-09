#ifndef ATTRACTS_BRIDGE__TRANSCEIVER_MODULE_BRIDGE_NODE_HPP_
#define ATTRACTS_BRIDGE__TRANSCEIVER_MODULE_BRIDGE_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <attracts_msgs/msg/game_data_input.hpp>
#include <attracts_msgs/msg/game_data_robot.hpp>

class TransceiverModuleBridge : public rclcpp::Node
{
public:
  static constexpr int BUFFER_SIZE = 1500;
  static constexpr double CB_RATE = 20.0;  // Hz

  TransceiverModuleBridge();

  static uint8_t CalculateCRC8(const uint8_t * data, size_t length);
  static uint16_t CalculateCRC16(const uint8_t * data, size_t length);
  static void ProcessFrameData(
    uint8_t command_type, uint8_t * data, uint16_t data_length,
    attracts_msgs::msg::GameDataInput & game_data_input,
    attracts_msgs::msg::GameDataRobot & game_data_robot,
    int & control_lost_counter);
  static void ProcessBuffer(
    uint8_t * buffer, int & buffer_index,
    attracts_msgs::msg::GameDataInput & game_data_input,
    attracts_msgs::msg::GameDataRobot & game_data_robot,
    int & control_lost_counter);

private:
  int OpenSerialPort(const std::string & device_name);
  void TimerCB();
  static void DropOneByte(uint8_t * buffer, int & buffer_index);

  std::string device_name_;
  int fd1_ = -1;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<attracts_msgs::msg::GameDataInput>::SharedPtr game_data_input_pub_;
  rclcpp::Publisher<attracts_msgs::msg::GameDataRobot>::SharedPtr game_data_robot_pub_;

  uint8_t buffer_[BUFFER_SIZE];
  int buffer_index_;
  int control_lost_counter_;

  attracts_msgs::msg::GameDataInput game_data_input_;
  attracts_msgs::msg::GameDataRobot game_data_robot_;
};

#endif  // ATTRACTS_BRIDGE__TRANSCEIVER_MODULE_BRIDGE_NODE_HPP_
