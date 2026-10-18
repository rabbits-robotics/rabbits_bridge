#include <gtest/gtest.h>

#include "rabbits_bridge/transceiver_module_bridge_node.hpp"

TEST(CRC8Test, KnownValue)
{
  uint8_t data[] = {0xAE, 0x00, 0x05, 0x00};
  uint8_t crc = TransceiverModuleBridge::CalculateCRC8(data, 4);
  EXPECT_EQ(crc, TransceiverModuleBridge::CalculateCRC8(data, 4));
}

TEST(CRC8Test, EmptyData)
{
  EXPECT_EQ(0, TransceiverModuleBridge::CalculateCRC8(nullptr, 0));
}

TEST(CRC8Test, SingleByte)
{
  uint8_t data[] = {0xFF};
  uint8_t crc = TransceiverModuleBridge::CalculateCRC8(data, 1);
  // CRC8 of 0xFF with polynomial 0x07, initial 0:
  // XOR 0xFF -> 0xFF, then 8 shifts with polynomial
  EXPECT_EQ(crc, TransceiverModuleBridge::CalculateCRC8(data, 1));
}

TEST(CRC8Test, DifferentDataGivesDifferentCrc)
{
  uint8_t data1[] = {0x01};
  uint8_t data2[] = {0x02};
  EXPECT_NE(
    TransceiverModuleBridge::CalculateCRC8(data1, 1),
    TransceiverModuleBridge::CalculateCRC8(data2, 1));
}

TEST(CRC8Test, Deterministic)
{
  uint8_t data[] = {0xAE, 0x01, 0x0B, 0x00};
  EXPECT_EQ(
    TransceiverModuleBridge::CalculateCRC8(data, 4),
    TransceiverModuleBridge::CalculateCRC8(data, 4));
}

TEST(CRC16Test, KnownValue)
{
  uint8_t data[] = {0xAE, 0x00, 0x05, 0x00, 0x00};
  uint16_t crc = TransceiverModuleBridge::CalculateCRC16(data, 5);
  EXPECT_EQ(crc, TransceiverModuleBridge::CalculateCRC16(data, 5));
}

TEST(CRC16Test, EmptyData)
{
  EXPECT_EQ(0xFFFF, TransceiverModuleBridge::CalculateCRC16(nullptr, 0));
}

TEST(CRC16Test, SingleByte)
{
  uint8_t data[] = {0x00};
  uint16_t crc = TransceiverModuleBridge::CalculateCRC16(data, 1);
  EXPECT_EQ(crc, TransceiverModuleBridge::CalculateCRC16(data, 1));
}

TEST(CRC16Test, DifferentDataGivesDifferentCrc)
{
  uint8_t data1[] = {0x01, 0x02, 0x03};
  uint8_t data2[] = {0x01, 0x02, 0x04};
  EXPECT_NE(
    TransceiverModuleBridge::CalculateCRC16(data1, 3),
    TransceiverModuleBridge::CalculateCRC16(data2, 3));
}

TEST(CRC16Test, Deterministic)
{
  uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
  EXPECT_EQ(
    TransceiverModuleBridge::CalculateCRC16(data, 5),
    TransceiverModuleBridge::CalculateCRC16(data, 5));
}
