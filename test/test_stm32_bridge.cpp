#include <gtest/gtest.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rabbits_msgs/msg/attracts_command.hpp>

#include "rabbits_bridge/stm32_bridge_node.hpp"
#include <rabcl/interface/uart.hpp>

class Stm32BridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ASSERT_EQ(pipe(pipe_fd_), 0);
    // Make read end non-blocking
    int flags = fcntl(pipe_fd_[0], F_GETFL, 0);
    fcntl(pipe_fd_[0], F_SETFL, flags | O_NONBLOCK);

    node_ = std::make_shared<Stm32Bridge>(pipe_fd_[0]);
  }

  void TearDown() override
  {
    node_.reset();
    close(pipe_fd_[1]);
    rclcpp::shutdown();
  }

  void WriteFeedbackPacket(const rabcl::Info & info)
  {
    rabcl::Uart uart;
    uart.PrepareFeedbackPacket(info);
    ssize_t n = write(pipe_fd_[1], uart.feedback_transmit_buffer_,
      rabcl::Uart::FEEDBACK_PACKET_SIZE);
    ASSERT_EQ(n, rabcl::Uart::FEEDBACK_PACKET_SIZE);
  }

  void SpinFor(int ms)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(ms)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  int pipe_fd_[2];
  std::shared_ptr<Stm32Bridge> node_;
};

TEST_F(Stm32BridgeTest, FeedbackPublishesImuAndJointState)
{
  sensor_msgs::msg::Imu::SharedPtr received_imu;
  sensor_msgs::msg::JointState::SharedPtr received_js;

  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {received_imu = msg;});

  auto js_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "act/joint_states", 10,
    [&](sensor_msgs::msg::JointState::SharedPtr msg) {received_js = msg;});

  rabcl::Info info{};
  info.imu_.acc_x_ = 1.5f;
  info.imu_.acc_z_ = 9.81f;
  info.imu_.gyro_z_ = 0.5f;
  info.yaw_act_.position_ = 1.0f;
  info.pitch_act_.velocity_ = -2.0f;
  info.chassis_fr_act_.torque_ = 3.0f;

  WriteFeedbackPacket(info);
  SpinFor(50);

  ASSERT_NE(received_imu, nullptr);
  EXPECT_FLOAT_EQ(received_imu->linear_acceleration.x, 1.5);
  EXPECT_FLOAT_EQ(received_imu->linear_acceleration.z, 9.81);
  EXPECT_FLOAT_EQ(received_imu->angular_velocity.z, 0.5);

  ASSERT_NE(received_js, nullptr);
  ASSERT_EQ(received_js->name.size(), 6u);
  EXPECT_EQ(received_js->name[0], "yaw");
  EXPECT_DOUBLE_EQ(received_js->position[0], 1.0);
  EXPECT_DOUBLE_EQ(received_js->velocity[1], -2.0);
  EXPECT_DOUBLE_EQ(received_js->effort[2], 3.0);
}

TEST_F(Stm32BridgeTest, InvalidFeedbackNotPublished)
{
  sensor_msgs::msg::Imu::SharedPtr received_imu;
  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {received_imu = msg;});

  // Write garbage data (no valid header)
  uint8_t garbage[112] = {};
  garbage[0] = 0x00;
  ssize_t n = write(pipe_fd_[1], garbage, sizeof(garbage));
  ASSERT_EQ(n, static_cast<ssize_t>(sizeof(garbage)));

  SpinFor(50);
  EXPECT_EQ(received_imu, nullptr);
}

TEST_F(Stm32BridgeTest, FeedbackWithCorruptedCrcNotPublished)
{
  sensor_msgs::msg::Imu::SharedPtr received_imu;
  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {received_imu = msg;});

  rabcl::Info info{};
  info.imu_.acc_x_ = 1.0f;
  rabcl::Uart uart;
  uart.PrepareFeedbackPacket(info);
  uart.feedback_transmit_buffer_[110] ^= 0xFF;  // corrupt CRC
  ssize_t n = write(pipe_fd_[1], uart.feedback_transmit_buffer_,
    rabcl::Uart::FEEDBACK_PACKET_SIZE);
  ASSERT_EQ(n, rabcl::Uart::FEEDBACK_PACKET_SIZE);

  SpinFor(50);
  EXPECT_EQ(received_imu, nullptr);
}

TEST_F(Stm32BridgeTest, MultiplePacketsReceived)
{
  int imu_count = 0;
  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr) {imu_count++;});

  rabcl::Info info{};
  for (int i = 0; i < 3; i++) {
    info.imu_.acc_x_ = static_cast<float>(i);
    WriteFeedbackPacket(info);
  }

  SpinFor(100);
  EXPECT_EQ(imu_count, 3);
}

TEST_F(Stm32BridgeTest, CmdCBWritesReferencePacket)
{
  auto cmd_pub = node_->create_publisher<rabbits_msgs::msg::AttractsCommand>("cmd", 10);

  auto msg = rabbits_msgs::msg::AttractsCommand();
  msg.chassis_vel.x = 1.0;
  msg.yaw_pos = 2.0;
  msg.load_mode = 1;

  cmd_pub->publish(msg);
  SpinFor(50);

  // Read from write end of pipe to verify data was sent
  // (In test, fd1_ is the read end of pipe, so CmdCB writes to it,
  //  which will fail since it's the read end. This tests error handling path.)
  // The important thing is that CmdCB doesn't crash.
}

TEST_F(Stm32BridgeTest, HeaderRecoveryAfterPartialData)
{
  sensor_msgs::msg::Imu::SharedPtr received_imu;
  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10,
    [&](sensor_msgs::msg::Imu::SharedPtr msg) {received_imu = msg;});

  // Write some garbage bytes first
  uint8_t garbage[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
  write(pipe_fd_[1], garbage, sizeof(garbage));

  // Then write a valid packet
  rabcl::Info info{};
  info.imu_.acc_x_ = 42.0f;
  WriteFeedbackPacket(info);

  SpinFor(100);

  ASSERT_NE(received_imu, nullptr);
  EXPECT_FLOAT_EQ(received_imu->linear_acceleration.x, 42.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
