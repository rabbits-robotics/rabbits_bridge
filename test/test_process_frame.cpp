#include <gtest/gtest.h>

#include <vector>

#include "attracts_bridge/transceiver_module_bridge_node.hpp"

// Helper to call ProcessFrameData with fresh output structs
static attracts_msgs::msg::GameDataInput ProcessInput(uint8_t * data, uint16_t len)
{
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 42;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, len, input, robot, counter);
  return input;
}

static attracts_msgs::msg::GameDataRobot ProcessRobot(uint8_t * data, uint16_t len)
{
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 0;
  TransceiverModuleBridge::ProcessFrameData(0x01, data, len, input, robot, counter);
  return robot;
}

// struct DataInput (packed): int16 x, int16 y, uint8 buttons, kb1, kb2, kb3, kb4 = 9 bytes
// data_length check in ProcessFrameData is == 9

TEST(ProcessFrameDataTest, InputMouseDelta)
{
  // mouse_delta_x = 0x0102 = 258, mouse_delta_y = 0xFF00 = -256 (int16 little-endian)
  uint8_t data[9] = {0x02, 0x01, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 9);
  EXPECT_EQ(258, out.mouse_delta_x);
  EXPECT_EQ(-256, out.mouse_delta_y);
}

TEST(ProcessFrameDataTest, InputMouseButtonsAllSet)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 9);
  EXPECT_TRUE(out.mouse_left_button);
  EXPECT_TRUE(out.mouse_right_button);
  EXPECT_TRUE(out.mouse_wheel_button);
  EXPECT_TRUE(out.mouse_side_1_button);
  EXPECT_TRUE(out.mouse_side_2_button);
}

TEST(ProcessFrameDataTest, InputMouseButtonsNone)
{
  uint8_t data[9] = {};
  auto out = ProcessInput(data, 9);
  EXPECT_FALSE(out.mouse_left_button);
  EXPECT_FALSE(out.mouse_right_button);
  EXPECT_FALSE(out.mouse_wheel_button);
  EXPECT_FALSE(out.mouse_side_1_button);
  EXPECT_FALSE(out.mouse_side_2_button);
}

TEST(ProcessFrameDataTest, InputKeyboard1)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 9);
  EXPECT_TRUE(out.key_1);
  EXPECT_TRUE(out.key_2);
  EXPECT_TRUE(out.key_3);
  EXPECT_TRUE(out.key_4);
  EXPECT_TRUE(out.key_5);
  EXPECT_TRUE(out.key_tab);
  EXPECT_TRUE(out.key_q);
  EXPECT_TRUE(out.key_w);
}

TEST(ProcessFrameDataTest, InputKeyboard2)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00};
  auto out = ProcessInput(data, 9);
  EXPECT_TRUE(out.key_e);
  EXPECT_TRUE(out.key_r);
  EXPECT_TRUE(out.key_t);
  EXPECT_TRUE(out.key_a);
  EXPECT_TRUE(out.key_s);
  EXPECT_TRUE(out.key_d);
  EXPECT_TRUE(out.key_f);
  EXPECT_TRUE(out.key_g);
}

TEST(ProcessFrameDataTest, InputKeyboard3)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00};
  auto out = ProcessInput(data, 9);
  EXPECT_TRUE(out.key_h);
  EXPECT_TRUE(out.key_shift);
  EXPECT_TRUE(out.key_z);
  EXPECT_TRUE(out.key_x);
  EXPECT_TRUE(out.key_c);
  EXPECT_TRUE(out.key_v);
  EXPECT_TRUE(out.key_b);
}

TEST(ProcessFrameDataTest, InputKeyboard4)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F};
  auto out = ProcessInput(data, 9);
  EXPECT_TRUE(out.key_ctrl);
  EXPECT_TRUE(out.key_alt);
  EXPECT_TRUE(out.key_space);
  EXPECT_TRUE(out.key_enter);
}

TEST(ProcessFrameDataTest, InputResetsControlLostCounter)
{
  uint8_t data[9] = {};
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 999;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, 9, input, robot, counter);
  EXPECT_EQ(0, counter);
}

TEST(ProcessFrameDataTest, InputWrongLengthSkipped)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
  int counter = 42;
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, 8, input, robot, counter);
  // data_length != 9 so not processed — counter unchanged
  EXPECT_EQ(42, counter);
}

// struct DataRobot (packed): uint8 type, team, speed_max, uint16 max_hp, cur_hp, max_heat, cur_heat
// = 3 + 8 = 11 bytes

TEST(ProcessFrameDataTest, RobotDataAllFields)
{
  // type=1, team=0, speed_max=30, max_hp=0x0400=1024, cur_hp=0x0300=768,
  // max_heat=0x0500=1280, cur_heat=0x0200=512
  uint8_t data[11] = {0x01, 0x00, 0x1E, 0x00, 0x04, 0x00, 0x03, 0x00, 0x05, 0x00, 0x02};
  auto out = ProcessRobot(data, 11);
  EXPECT_EQ(1, out.type);
  EXPECT_EQ(0, out.team);
  EXPECT_EQ(30, out.projectile_speed_max);
  EXPECT_EQ(1024, out.max_hp);
  EXPECT_EQ(768, out.current_hp);
  EXPECT_EQ(1280, out.max_heat);
  EXPECT_EQ(512, out.current_heat);
}

TEST(ProcessFrameDataTest, RobotDataWrongLengthSkipped)
{
  uint8_t data[11] = {0x01, 0x00, 0x1E, 0x00, 0x04, 0x00, 0x03, 0x00, 0x05, 0x00, 0x02};
  auto out = ProcessRobot(data, 10);
  // data_length != 11 so not processed — fields remain default (0)
  EXPECT_EQ(0, out.type);
}

TEST(ProcessFrameDataTest, UnknownCommandTypeNoOp)
{
  uint8_t data[1] = {0x00};
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 7;
  TransceiverModuleBridge::ProcessFrameData(0xFF, data, 1, input, robot, counter);
  EXPECT_EQ(7, counter);  // unchanged
}

// ---- ProcessBuffer (frame framing / multi-frame / resync) ----

// Build a full frame: 0xAE, type, len(LE), CRC8(header4), payload..., CRC16(LE)
static std::vector<uint8_t> BuildFrame(
  uint8_t command_type, const std::vector<uint8_t> & payload)
{
  uint16_t len = static_cast<uint16_t>(payload.size());
  std::vector<uint8_t> frame;
  frame.push_back(0xAE);
  frame.push_back(command_type);
  frame.push_back(static_cast<uint8_t>(len & 0xFF));
  frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
  frame.push_back(TransceiverModuleBridge::CalculateCRC8(frame.data(), 4));
  frame.insert(frame.end(), payload.begin(), payload.end());
  uint16_t crc16 = TransceiverModuleBridge::CalculateCRC16(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc16 & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc16 >> 8) & 0xFF));
  return frame;
}

// Input payload: mouse_delta_x=+10, mouse_delta_y=-5, left button + W key
static std::vector<uint8_t> SampleInputPayload()
{
  return {0x0A, 0x00, 0xFB, 0xFF, 0x01, 0x80, 0x00, 0x00, 0x00};
}

// Robot payload: type=0, team=0, speed=20, maxHP=100, curHP=100, maxHeat=200, curHeat=50
static std::vector<uint8_t> SampleRobotPayload()
{
  return {0x00, 0x00, 0x14, 0x64, 0x00, 0x64, 0x00, 0xC8, 0x00, 0x32, 0x00};
}

TEST(ProcessBufferTest, SingleValidInputFrameConsumed)
{
  auto frame = BuildFrame(0x00, SampleInputPayload());
  int buffer_index = static_cast<int>(frame.size());
  uint8_t buffer[1500];
  std::copy(frame.begin(), frame.end(), buffer);

  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 999;
  TransceiverModuleBridge::ProcessBuffer(buffer, buffer_index, input, robot, counter);

  EXPECT_EQ(0, buffer_index);  // フレームを消費し尽くす
  EXPECT_EQ(10, input.mouse_delta_x);
  EXPECT_EQ(-5, input.mouse_delta_y);
  EXPECT_TRUE(input.mouse_left_button);
  EXPECT_TRUE(input.key_w);
  EXPECT_EQ(0, counter);  // 入力フレーム受信で通信正常
}

TEST(ProcessBufferTest, TwoConcatenatedFramesBothConsumed)
{
  auto f1 = BuildFrame(0x00, SampleInputPayload());
  auto f2 = BuildFrame(0x01, SampleRobotPayload());
  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), f1.begin(), f1.end());
  bytes.insert(bytes.end(), f2.begin(), f2.end());

  int buffer_index = static_cast<int>(bytes.size());
  uint8_t buffer[1500];
  std::copy(bytes.begin(), bytes.end(), buffer);

  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 0;
  TransceiverModuleBridge::ProcessBuffer(buffer, buffer_index, input, robot, counter);

  EXPECT_EQ(0, buffer_index);  // 連結された2フレームを両方処理
  EXPECT_EQ(10, input.mouse_delta_x);
  EXPECT_EQ(20, robot.projectile_speed_max);
  EXPECT_EQ(100, robot.current_hp);
}

TEST(ProcessBufferTest, PartialFrameRetained)
{
  auto frame = BuildFrame(0x00, SampleInputPayload());
  // 末尾2バイト（フレーム未完）を欠いた状態
  int partial = static_cast<int>(frame.size()) - 2;
  int buffer_index = partial;
  uint8_t buffer[1500];
  std::copy(frame.begin(), frame.begin() + partial, buffer);

  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 5;
  TransceiverModuleBridge::ProcessBuffer(buffer, buffer_index, input, robot, counter);

  // 未完フレームはバッファに残し、消費しない
  EXPECT_EQ(partial, buffer_index);
  EXPECT_EQ(5, counter);  // まだ処理していない
}

TEST(ProcessBufferTest, LeadingGarbageResyncs)
{
  auto frame = BuildFrame(0x00, SampleInputPayload());
  std::vector<uint8_t> bytes = {0x12, 0x34, 0xFF};  // 先頭ノイズ
  bytes.insert(bytes.end(), frame.begin(), frame.end());

  int buffer_index = static_cast<int>(bytes.size());
  uint8_t buffer[1500];
  std::copy(bytes.begin(), bytes.end(), buffer);

  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 999;
  TransceiverModuleBridge::ProcessBuffer(buffer, buffer_index, input, robot, counter);

  EXPECT_EQ(0, buffer_index);  // ノイズを1バイトずつ送って再同期し処理
  EXPECT_EQ(10, input.mouse_delta_x);
  EXPECT_EQ(0, counter);
}

TEST(ProcessBufferTest, BadCrc16FrameDropped)
{
  auto frame = BuildFrame(0x00, SampleInputPayload());
  frame[frame.size() - 1] ^= 0xFF;  // CRC-16を破壊
  int buffer_index = static_cast<int>(frame.size());
  uint8_t buffer[1500];
  std::copy(frame.begin(), frame.end(), buffer);

  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 5;
  TransceiverModuleBridge::ProcessBuffer(buffer, buffer_index, input, robot, counter);

  // 不正フレームは消費されず（1バイトずつ送って再同期試行）入力は更新されない
  EXPECT_EQ(0, input.mouse_delta_x);
  EXPECT_EQ(5, counter);
}
