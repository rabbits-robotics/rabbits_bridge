// robot_bridge: rabcl の共通プロトコル（rabcl::protocol）でファームとシリアル通信する汎用ノード。
// 機体の違いはパラメータ（device / baud / joints / modes）だけ。関節名は URDF と同じにする。
//   出力: joint_states (sensor_msgs/JointState), imu/data (sensor_msgs/Imu)
//   入力: reference_command (rabbits_msgs/ReferenceCommand)
#ifndef RABBITS_BRIDGE__ROBOT_BRIDGE_NODE_HPP_
#define RABBITS_BRIDGE__ROBOT_BRIDGE_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rabbits_msgs/msg/reference_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rabcl/protocol/parser.hpp>
#include <rabcl/protocol/protocol.hpp>

class RobotBridge : public rclcpp::Node
{
public:
  explicit RobotBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  // テスト用: 開いた fd（socketpair など）を使う
  RobotBridge(int test_fd, const rclcpp::NodeOptions & options);
  ~RobotBridge();

private:
  void Init();
  int OpenSerialPort(const std::string & device, int baud);
  void ReadTimerCallback();
  void ReferenceCallback(const rabbits_msgs::msg::ReferenceCommand::SharedPtr msg);
  // フレームを送信待ちに積み、書けるだけ書く（ブロックしない）
  void WriteFrame(const uint8_t * buf, size_t len);
  // 送信待ちを書けるだけ書く。EAGAIN なら残して次回に回す
  void FlushTx();
  static void OnFrameStatic(
    rabcl::protocol::Type type, const uint8_t * payload, uint8_t len, void * ctx);
  void OnFrame(rabcl::protocol::Type type, const uint8_t * payload, uint8_t len);

  rclcpp::Subscription<rabbits_msgs::msg::ReferenceCommand>::SharedPtr reference_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  std::vector<std::string> joints_;
  std::vector<std::string> modes_;
  int fd_ = -1;
  rabcl::protocol::Parser parser_;
  std::vector<uint8_t> tx_pending_;  // まだ書けていない送信バイト列
  static constexpr size_t TX_PENDING_MAX = 64 * 1024;  // これを超えたら古い分を捨てる
  bool version_error_logged_ = false;
  bool joint_count_warned_ = false;
  bool unknown_joint_warned_ = false;
};

#endif  // RABBITS_BRIDGE__ROBOT_BRIDGE_NODE_HPP_
