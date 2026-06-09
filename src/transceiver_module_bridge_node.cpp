#include "attracts_bridge/transceiver_module_bridge_node.hpp"

#include <functional>

TransceiverModuleBridge::TransceiverModuleBridge()
: Node("transceiver_module_bridge_node"), buffer_index_(0), control_lost_counter_(0)
{
  device_name_ = "/dev/ttyUSB0";
  fd1_ = OpenSerialPort(device_name_);
  if (fd1_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
    rclcpp::shutdown();
  }

  game_data_input_pub_ =
    this->create_publisher<attracts_msgs::msg::GameDataInput>("game_data_input", 10);
  game_data_robot_pub_ =
    this->create_publisher<attracts_msgs::msg::GameDataRobot>("game_data_robot", 10);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / CB_RATE),
    std::bind(&TransceiverModuleBridge::TimerCB, this));
}

int TransceiverModuleBridge::OpenSerialPort(const std::string & device_name)
{
  int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY);

  fcntl(fd1, F_SETFL, FNDELAY);
  struct termios conf_tio;
  tcgetattr(fd1, &conf_tio);

  speed_t baudrate = B115200;
  cfsetispeed(&conf_tio, baudrate);
  cfsetospeed(&conf_tio, baudrate);

  conf_tio.c_cflag &= ~PARENB;         // パリティなし
  conf_tio.c_cflag &= ~CSTOPB;         // ストップビット1
  conf_tio.c_cflag &= ~CSIZE;
  conf_tio.c_cflag |= CS8;             // データビット8
  conf_tio.c_cflag |= CLOCAL | CREAD;  // ローカル接続・受信有効
  conf_tio.c_cflag &= ~CRTSCTS;        // ハードウェアフロー制御なし

  conf_tio.c_iflag = IGNPAR;           // パリティエラー無視

  conf_tio.c_lflag &= ~(ECHO | ICANON);

  tcflush(fd1, TCIOFLUSH);
  tcsetattr(fd1, TCSANOW, &conf_tio);

  return fd1;
}

void TransceiverModuleBridge::TimerCB()
{
  // タイムアウト処理
  ++control_lost_counter_;
  if (control_lost_counter_ > CB_RATE * CONTROL_LOST_TIMEOUT) {
    // 全入力をリセット
    game_data_input_ = attracts_msgs::msg::GameDataInput();
  }

  // 情報を送信
  game_data_input_pub_->publish(game_data_input_);
  game_data_robot_pub_->publish(game_data_robot_);

  // シリアルデータ受信（バッファ末尾へ追記）
  if (buffer_index_ >= BUFFER_SIZE - 1) {
    // バッファ満杯フェイルセーフ: 同期不能とみなしてリセット
    buffer_index_ = 0;
  }
  int n = read(fd1_, buffer_ + buffer_index_, BUFFER_SIZE - 1 - buffer_index_);
  if (n > 0) {
    buffer_index_ += n;
  }

  // バッファ内の完全なフレームをすべて処理
  ProcessBuffer(
    buffer_, buffer_index_, game_data_input_, game_data_robot_, control_lost_counter_);
}

void TransceiverModuleBridge::DropOneByte(uint8_t * buffer, int & buffer_index)
{
  for (int i = 0; i < buffer_index - 1; ++i) {
    buffer[i] = buffer[i + 1];
  }
  --buffer_index;
}

void TransceiverModuleBridge::ProcessBuffer(
  uint8_t * buffer, int & buffer_index,
  attracts_msgs::msg::GameDataInput & game_data_input,
  attracts_msgs::msg::GameDataRobot & game_data_robot,
  int & control_lost_counter)
{
  while (buffer_index >= 7) {
    uint8_t start_of_frame = buffer[0];
    uint8_t command_type = buffer[1];
    uint16_t data_length = buffer[2] + (buffer[3] << 8);  // リトルエンディアン
    uint8_t crc8 = buffer[4];

    // ヘッダー検証
    if (start_of_frame != 0xAE || data_length > 993 ||
      CalculateCRC8(buffer, 4) != crc8)
    {
      DropOneByte(buffer, buffer_index);
      continue;
    }

    // フレーム未完: 追記を待つ
    int frame_size = data_length + 7;
    if (buffer_index < frame_size) {
      break;
    }

    // CRC-16検証（フレーム末尾の2バイト、フレーム境界で計算）
    uint16_t crc16 = buffer[frame_size - 2] + (buffer[frame_size - 1] << 8);
    if (CalculateCRC16(buffer, frame_size - 2) != crc16) {
      DropOneByte(buffer, buffer_index);
      continue;
    }

    // 正常フレーム処理
    ProcessFrameData(
      command_type, buffer + 5, data_length,
      game_data_input, game_data_robot, control_lost_counter);

    // 処理済みフレームをバッファから除去
    for (int i = 0; i < buffer_index - frame_size; ++i) {
      buffer[i] = buffer[i + frame_size];
    }
    buffer_index -= frame_size;
  }
}

uint8_t TransceiverModuleBridge::CalculateCRC8(const uint8_t * data, size_t length)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint16_t TransceiverModuleBridge::CalculateCRC16(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void TransceiverModuleBridge::ProcessFrameData(
  uint8_t command_type, uint8_t * data, uint16_t data_length,
  attracts_msgs::msg::GameDataInput & game_data_input,
  attracts_msgs::msg::GameDataRobot & game_data_robot,
  int & control_lost_counter)
{
  if (command_type == 0x00 && data_length == 9) {
    // 入力データ処理
    struct DataInput
    {
      int16_t mouse_delta_x;
      int16_t mouse_delta_y;
      uint8_t mouse_buttons;
      uint8_t keyboard1;
      uint8_t keyboard2;
      uint8_t keyboard3;
      uint8_t keyboard4;
    } __attribute__((__packed__));

    DataInput * input = reinterpret_cast<DataInput *>(data);

    // マウス移動
    game_data_input.mouse_delta_x = input->mouse_delta_x;
    game_data_input.mouse_delta_y = input->mouse_delta_y;
    // マウスボタン
    game_data_input.mouse_left_button = input->mouse_buttons & 0x01;
    game_data_input.mouse_right_button = input->mouse_buttons & 0x02;
    game_data_input.mouse_wheel_button = input->mouse_buttons & 0x04;
    game_data_input.mouse_side_1_button = input->mouse_buttons & 0x08;
    game_data_input.mouse_side_2_button = input->mouse_buttons & 0x10;
    // キーボード - バイト1 (数字キー + Tab, Q, W)
    game_data_input.key_1 = input->keyboard1 & 0x01;
    game_data_input.key_2 = input->keyboard1 & 0x02;
    game_data_input.key_3 = input->keyboard1 & 0x04;
    game_data_input.key_4 = input->keyboard1 & 0x08;
    game_data_input.key_5 = input->keyboard1 & 0x10;
    game_data_input.key_tab = input->keyboard1 & 0x20;
    game_data_input.key_q = input->keyboard1 & 0x40;
    game_data_input.key_w = input->keyboard1 & 0x80;
    // キーボード - バイト2 (E, R, T, A, S, D, F, G)
    game_data_input.key_e = input->keyboard2 & 0x01;
    game_data_input.key_r = input->keyboard2 & 0x02;
    game_data_input.key_t = input->keyboard2 & 0x04;
    game_data_input.key_a = input->keyboard2 & 0x08;
    game_data_input.key_s = input->keyboard2 & 0x10;
    game_data_input.key_d = input->keyboard2 & 0x20;
    game_data_input.key_f = input->keyboard2 & 0x40;
    game_data_input.key_g = input->keyboard2 & 0x80;
    // キーボード - バイト3 (H, Shift, Z, X, C, V, B)
    game_data_input.key_h = input->keyboard3 & 0x01;
    game_data_input.key_shift = input->keyboard3 & 0x02;
    game_data_input.key_z = input->keyboard3 & 0x04;
    game_data_input.key_x = input->keyboard3 & 0x08;
    game_data_input.key_c = input->keyboard3 & 0x10;
    game_data_input.key_v = input->keyboard3 & 0x20;
    game_data_input.key_b = input->keyboard3 & 0x40;
    // キーボード - バイト4 (Ctrl, Alt, Space, Enter)
    game_data_input.key_ctrl = input->keyboard4 & 0x01;
    game_data_input.key_alt = input->keyboard4 & 0x02;
    game_data_input.key_space = input->keyboard4 & 0x04;
    game_data_input.key_enter = input->keyboard4 & 0x08;

    control_lost_counter = 0;  // 通信正常
  } else if (command_type == 0x01 && data_length == 11) {
    // ロボット情報処理
    struct DataRobot
    {
      uint8_t type;
      uint8_t team;
      uint8_t projectile_speed_max;
      uint16_t max_hp;
      uint16_t current_hp;
      uint16_t max_heat;
      uint16_t current_heat;
    } __attribute__((__packed__));

    DataRobot * robot = reinterpret_cast<DataRobot *>(data);
    game_data_robot.type = robot->type;
    game_data_robot.team = robot->team;
    game_data_robot.projectile_speed_max = robot->projectile_speed_max;
    game_data_robot.max_hp = robot->max_hp;
    game_data_robot.current_hp = robot->current_hp;
    game_data_robot.max_heat = robot->max_heat;
    game_data_robot.current_heat = robot->current_heat;
  }
}

#ifndef ATTRACTS_BRIDGE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransceiverModuleBridge>());
  rclcpp::shutdown();
  return 0;
}
#endif
