#include "rabbits_bridge/robot_bridge_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <functional>

#include <rabcl_ros2/convert.hpp>

namespace proto = rabcl::protocol;

namespace
{

speed_t ToSpeed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return 0;
  }
}

}  // namespace

RobotBridge::RobotBridge(const rclcpp::NodeOptions & options)
: Node("robot_bridge", options)
{
  const auto device = this->declare_parameter<std::string>("device", "/dev/ttyACM0");
  const auto baud = static_cast<int>(this->declare_parameter<int64_t>("baud", 230400));
  fd_ = OpenSerialPort(device, baud);
  if (fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "could not open %s", device.c_str());
    rclcpp::shutdown();
    return;
  }
  Init();
}

RobotBridge::RobotBridge(int test_fd, const rclcpp::NodeOptions & options)
: Node("robot_bridge", options), fd_(test_fd)
{
  this->declare_parameter<std::string>("device", "");
  this->declare_parameter<int64_t>("baud", 0);
  Init();
}

RobotBridge::~RobotBridge()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void RobotBridge::Init()
{
  joints_ = this->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  modes_ = this->declare_parameter<std::vector<std::string>>("modes", std::vector<std::string>{});

  reference_sub_ = this->create_subscription<rabbits_msgs::msg::ReferenceCommand>(
    "reference_command", 10,
    std::bind(&RobotBridge::ReferenceCallback, this, std::placeholders::_1));
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  read_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&RobotBridge::ReadTimerCallback, this));
}

int RobotBridge::OpenSerialPort(const std::string & device, int baud)
{
  const speed_t speed = ToSpeed(baud);
  if (speed == 0) {
    RCLCPP_ERROR(this->get_logger(), "unsupported baud rate %d", baud);
    return -1;
  }
  const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }
  struct termios tio;
  std::memset(&tio, 0, sizeof(tio));
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &tio);
  return fd;
}

void RobotBridge::ReadTimerCallback()
{
  uint8_t buf[512];
  while (true) {
    const ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      return;
    }
    parser_.Feed(buf, static_cast<size_t>(n), &RobotBridge::OnFrameStatic, this);
  }
}

void RobotBridge::OnFrameStatic(
  proto::Type type, const uint8_t * payload, uint8_t len, void * ctx)
{
  static_cast<RobotBridge *>(ctx)->OnFrame(type, payload, len);
}

void RobotBridge::OnFrame(proto::Type type, const uint8_t * payload, uint8_t len)
{
  switch (type) {
    case proto::Type::INFO: {
        proto::Info info = {};
        if (!proto::DecodeInfo(payload, len, info)) {
          return;
        }
        if (info.protocol_version != proto::VERSION && !version_error_logged_) {
          RCLCPP_ERROR(
            this->get_logger(), "protocol version mismatch: firmware %u, this bridge %u",
            info.protocol_version, proto::VERSION);
          version_error_logged_ = true;
        }
        if (info.joint_count != joints_.size() && !joint_count_warned_) {
          RCLCPP_WARN(
            this->get_logger(), "joint count mismatch: firmware %u, parameter 'joints' %zu",
            info.joint_count, joints_.size());
          joint_count_warned_ = true;
        }
        break;
      }
    case proto::Type::JOINT_STATE: {
        proto::JointState joints[proto::MAX_JOINT_STATES];
        uint8_t count = 0;
        if (!proto::DecodeJointState(payload, len, joints, proto::MAX_JOINT_STATES, count)) {
          return;
        }
        sensor_msgs::msg::JointState msg;
        if (!rabcl_ros2::Convert::JointStatesToMsg(joints, count, joints_, msg) &&
          !unknown_joint_warned_)
        {
          RCLCPP_WARN(
            this->get_logger(), "received a joint id outside parameter 'joints' (%zu names)",
            joints_.size());
          unknown_joint_warned_ = true;
        }
        if (msg.name.empty()) {
          return;
        }
        msg.header.stamp = this->now();
        joint_state_pub_->publish(msg);
        break;
      }
    case proto::Type::IMU: {
        proto::Imu imu = {};
        if (!proto::DecodeImu(payload, len, imu)) {
          return;
        }
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        rabcl_ros2::Convert::ImuToMsg(imu, msg);
        imu_pub_->publish(msg);
        break;
      }
    default:
      // SYSTEM など、まだ扱わない種別は無視する
      break;
  }
}

void RobotBridge::ReferenceCallback(const rabbits_msgs::msg::ReferenceCommand::SharedPtr msg)
{
  if (!rabcl_ros2::Convert::ReferenceIsConsistent(*msg)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "reference_command dropped: joint_* arrays (or mode_* arrays) differ in length");
    return;
  }
  uint8_t buf[proto::MAX_FRAME];

  proto::ChassisVel vel = {};
  rabcl_ros2::Convert::ReferenceToChassisVel(*msg, vel);
  WriteFrame(buf, proto::EncodeChassisVel(vel, buf, sizeof(buf)));

  proto::JointCmd cmds[proto::MAX_JOINT_CMDS];
  const uint8_t n_cmds =
    rabcl_ros2::Convert::ReferenceToJointCmds(*msg, joints_, cmds, proto::MAX_JOINT_CMDS);
  if (n_cmds > 0) {
    WriteFrame(buf, proto::EncodeJointCmd(cmds, n_cmds, buf, sizeof(buf)));
  }

  proto::ModeCmd mode_cmds[proto::MAX_MODES];
  const uint8_t n_modes =
    rabcl_ros2::Convert::ReferenceToModes(*msg, modes_, mode_cmds, proto::MAX_MODES);
  if (n_modes > 0) {
    WriteFrame(buf, proto::EncodeMode(mode_cmds, n_modes, buf, sizeof(buf)));
  }
}

void RobotBridge::WriteFrame(const uint8_t * buf, size_t len)
{
  if (len == 0) {
    return;
  }
  const ssize_t n = write(fd_, buf, len);
  if (n < 0) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "write failed: %s", strerror(errno));
  } else if (static_cast<size_t>(n) != len) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "only %zd of %zu bytes written", n, len);
  } else if (isatty(fd_)) {
    tcdrain(fd_);
  }
}

#ifndef RABBITS_BRIDGE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotBridge>();
  if (rclcpp::ok()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
#endif
