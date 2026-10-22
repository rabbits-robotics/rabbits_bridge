#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rabbits_msgs/msg/reference_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rabcl/protocol/parser.hpp>
#include <rabcl/protocol/protocol.hpp>

#include "rabbits_bridge/robot_bridge_node.hpp"

namespace proto = rabcl::protocol;

class RobotBridgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fd_), 0);
    for (int fd : fd_) {
      fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"j0", "j1", "j2"}),
      rclcpp::Parameter("modes", std::vector<std::string>{"m0", "m1"}),
    });
    node_ = std::make_shared<RobotBridge>(fd_[0], options);
  }

  void TearDown() override
  {
    node_.reset();
    close(fd_[1]);
    rclcpp::shutdown();
  }

  // ファーム側（テスト）からフレームを書く
  void WriteBytes(const uint8_t * buf, size_t len)
  {
    ASSERT_EQ(write(fd_[1], buf, len), static_cast<ssize_t>(len));
  }

  void WriteJointState(const std::vector<proto::JointState> & joints)
  {
    uint8_t buf[proto::MAX_FRAME];
    const size_t n = proto::EncodeJointState(
      joints.data(), static_cast<uint8_t>(joints.size()), buf, sizeof(buf));
    ASSERT_GT(n, 0u);
    WriteBytes(buf, n);
  }

  using Frames = std::vector<std::pair<proto::Type, std::vector<uint8_t>>>;

  // bridge が書いて今読めるバイト列
  std::vector<uint8_t> ReadAvailable()
  {
    std::vector<uint8_t> bytes;
    uint8_t tmp[512];
    while (true) {
      const ssize_t n = read(fd_[1], tmp, sizeof(tmp));
      if (n <= 0) {break;}
      bytes.insert(bytes.end(), tmp, tmp + n);
    }
    return bytes;
  }

  static Frames ParseFrames(const std::vector<uint8_t> & bytes)
  {
    Frames frames;
    proto::Parser parser;
    parser.Feed(
      bytes.data(), bytes.size(),
      [](proto::Type type, const uint8_t * payload, uint8_t len, void * ctx) {
        static_cast<Frames *>(ctx)->emplace_back(
          type, std::vector<uint8_t>(payload, payload + len));
      }, &frames);
    return frames;
  }

  // bridge が書いたバイト列をフレームに分けて返す
  Frames ReadFrames() {return ParseFrames(ReadAvailable());}

  void SpinFor(int ms)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(ms)) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  rabbits_msgs::msg::ReferenceCommand MakeReference(double vx)
  {
    rabbits_msgs::msg::ReferenceCommand msg;
    msg.chassis_vel.x = vx;
    return msg;
  }

  int fd_[2];
  std::shared_ptr<RobotBridge> node_;
};

TEST_F(RobotBridgeTest, JointStateFramePublishesJointStates)
{
  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {received = msg;});

  WriteJointState({{0, 1.0f, 2.0f, 3.0f, 40.0f, 0}, {2, -1.0f, -2.0f, -3.0f, 30.0f, 0}});
  SpinFor(50);

  ASSERT_NE(received, nullptr);
  ASSERT_EQ(received->name.size(), 2u);
  EXPECT_EQ(received->name[0], "j0");
  EXPECT_EQ(received->name[1], "j2");
  EXPECT_DOUBLE_EQ(received->position[0], 1.0);
  EXPECT_DOUBLE_EQ(received->velocity[1], -2.0);
  EXPECT_DOUBLE_EQ(received->effort[1], -3.0);
}

TEST_F(RobotBridgeTest, ImuFramePublishesImu)
{
  sensor_msgs::msg::Imu::SharedPtr received;
  auto sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10, [&](sensor_msgs::msg::Imu::SharedPtr msg) {received = msg;});

  proto::Imu imu = {{1.5f, 0.0f, 9.81f}, {0.0f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.0f}};
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeImu(imu, buf, sizeof(buf));
  WriteBytes(buf, n);
  SpinFor(50);

  ASSERT_NE(received, nullptr);
  EXPECT_FLOAT_EQ(received->linear_acceleration.x, 1.5);
  EXPECT_FLOAT_EQ(received->linear_acceleration.z, 9.81);
  EXPECT_FLOAT_EQ(received->angular_velocity.z, 0.5);
  EXPECT_EQ(received->header.frame_id, "imu_link");
}

TEST_F(RobotBridgeTest, SystemFrameIsIgnored)
{
  int count = 0;
  auto js_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr) {count++;});
  auto imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10, [&](sensor_msgs::msg::Imu::SharedPtr) {count++;});

  proto::System system = {24.0f, 0, 1000};
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeSystem(system, buf, sizeof(buf));
  WriteBytes(buf, n);
  SpinFor(50);
  EXPECT_EQ(count, 0);
}

TEST_F(RobotBridgeTest, UnknownJointIdIsSkipped)
{
  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {received = msg;});

  WriteJointState({{5, 1.0f, 0, 0, 0, 0}, {1, 2.0f, 0, 0, 0, 0}, {0, 3.0f, 0, 0, 0, 0}});
  SpinFor(50);

  ASSERT_NE(received, nullptr);
  ASSERT_EQ(received->name.size(), 2u);
  EXPECT_EQ(received->name[0], "j1");
  EXPECT_EQ(received->name[1], "j0");
}

TEST_F(RobotBridgeTest, CorruptedCrcNotPublished)
{
  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {received = msg;});

  const proto::JointState joints[1] = {{0, 1.0f, 0, 0, 0, 0}};
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeJointState(joints, 1, buf, sizeof(buf));
  buf[n - 1] ^= 0xFF;
  WriteBytes(buf, n);
  SpinFor(50);
  EXPECT_EQ(received, nullptr);
}

TEST_F(RobotBridgeTest, GarbageThenFrameRecovers)
{
  sensor_msgs::msg::JointState::SharedPtr received;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {received = msg;});

  const uint8_t garbage[5] = {0x01, 0x02, 0xA5, 0x04, 0x05};
  WriteBytes(garbage, sizeof(garbage));
  WriteJointState({{1, 42.0f, 0, 0, 0, 0}});
  SpinFor(50);

  ASSERT_NE(received, nullptr);
  ASSERT_EQ(received->name.size(), 1u);
  EXPECT_EQ(received->name[0], "j1");
  EXPECT_DOUBLE_EQ(received->position[0], 42.0);
}

TEST_F(RobotBridgeTest, InfoVersionMismatchDoesNotCrash)
{
  int count = 0;
  auto sub = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr) {count++;});

  proto::Info info = {};
  info.protocol_version = 99;
  info.joint_count = 3;
  std::strncpy(info.firmware_id, "test", sizeof(info.firmware_id));
  uint8_t buf[proto::MAX_FRAME];
  const size_t n = proto::EncodeInfo(info, buf, sizeof(buf));
  WriteBytes(buf, n);
  SpinFor(50);
  EXPECT_EQ(count, 0);
}

TEST_F(RobotBridgeTest, ReferenceCommandWritesFrames)
{
  auto pub = node_->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);

  auto msg = MakeReference(1.0);
  msg.joint_name = {"j1"};
  msg.joint_position = {0.5f};
  msg.joint_velocity = {0.0f};
  msg.joint_effort = {0.0f};
  msg.joint_kp = {2.0f};
  msg.joint_kd = {0.1f};
  msg.mode_name = {"m1"};
  msg.mode_value = {2};
  pub->publish(msg);
  SpinFor(50);

  const auto frames = ReadFrames();
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_EQ(frames[0].first, proto::Type::CHASSIS_VEL);
  proto::ChassisVel vel = {};
  ASSERT_TRUE(proto::DecodeChassisVel(frames[0].second.data(), frames[0].second.size(), vel));
  EXPECT_FLOAT_EQ(vel.vx, 1.0f);

  EXPECT_EQ(frames[1].first, proto::Type::JOINT_CMD);
  proto::JointCmd cmds[4];
  uint8_t count = 0;
  ASSERT_TRUE(
    proto::DecodeJointCmd(frames[1].second.data(), frames[1].second.size(), cmds, 4, count));
  ASSERT_EQ(count, 1);
  EXPECT_EQ(cmds[0].id, 1);
  EXPECT_FLOAT_EQ(cmds[0].position, 0.5f);
  EXPECT_FLOAT_EQ(cmds[0].kp, 2.0f);

  EXPECT_EQ(frames[2].first, proto::Type::MODE);
  proto::ModeCmd modes[4];
  ASSERT_TRUE(proto::DecodeMode(frames[2].second.data(), frames[2].second.size(), modes, 4, count));
  ASSERT_EQ(count, 1);
  EXPECT_EQ(modes[0].id, 1);
  EXPECT_EQ(modes[0].value, 2);
}

TEST_F(RobotBridgeTest, EmptyArraysSendOnlyChassisVel)
{
  auto pub = node_->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  pub->publish(MakeReference(0.25));
  SpinFor(50);

  const auto frames = ReadFrames();
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].first, proto::Type::CHASSIS_VEL);
}

TEST_F(RobotBridgeTest, BackpressureKeepsEveryFrame)
{
  // ソケットの送信バッファを最小にして、読み出さないまま 200 回指令を出す。
  // write が EAGAIN になっても bridge がフレームを捨てず、あとで読み出せば全部届く
  int small = 1;
  ASSERT_EQ(setsockopt(fd_[0], SOL_SOCKET, SO_SNDBUF, &small, sizeof(small)), 0);
  ASSERT_EQ(setsockopt(fd_[1], SOL_SOCKET, SO_RCVBUF, &small, sizeof(small)), 0);

  auto pub = node_->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  constexpr int N = 200;
  for (int i = 0; i < N; ++i) {
    auto msg = MakeReference(static_cast<double>(i));
    msg.joint_name = {"j0"};
    msg.joint_position = {static_cast<float>(i)};
    msg.joint_velocity = {0.0f};
    msg.joint_effort = {0.0f};
    msg.joint_kp = {0.0f};
    msg.joint_kd = {0.0f};
    pub->publish(msg);
    SpinFor(2);
  }

  // 読み出しながら spin して、溜まった分を bridge に流させる。バイト列は溜めて最後に解析する
  std::vector<uint8_t> bytes;
  const size_t expected_bytes = static_cast<size_t>(N) * (17 + 5 + 1 + 21);  // CHASSIS_VEL + JOINT_CMD
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1500)) {
    const auto chunk = ReadAvailable();
    bytes.insert(bytes.end(), chunk.begin(), chunk.end());
    if (bytes.size() >= expected_bytes) {break;}
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto frames = ParseFrames(bytes);
  ASSERT_EQ(frames.size(), static_cast<size_t>(2 * N));
  // 順番も保たれる: 最後の CHASSIS_VEL の vx が N-1
  proto::ChassisVel vel = {};
  ASSERT_EQ(frames[2 * N - 2].first, proto::Type::CHASSIS_VEL);
  ASSERT_TRUE(
    proto::DecodeChassisVel(frames[2 * N - 2].second.data(), frames[2 * N - 2].second.size(), vel));
  EXPECT_FLOAT_EQ(vel.vx, static_cast<float>(N - 1));
}

TEST_F(RobotBridgeTest, MismatchedReferenceIsDropped)
{
  auto pub = node_->create_publisher<rabbits_msgs::msg::ReferenceCommand>("reference_command", 10);
  auto msg = MakeReference(1.0);
  msg.joint_name = {"j0", "j1"};
  msg.joint_position = {1.0f};
  pub->publish(msg);
  SpinFor(50);

  EXPECT_TRUE(ReadFrames().empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
