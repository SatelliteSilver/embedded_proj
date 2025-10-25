//
// PacketProtocol.h
// 16bit UART 데이터 패킷 정의 및 비트 해석 유틸
//

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <Arduino.h>

// 데이터 필드 구조 정의
struct PacketData {
  uint8_t speed_type;   // 2bit
  uint8_t speed_val;    // 3bit
  uint8_t accel_type;   // 2bit
  uint8_t accel_val;    // 3bit
  uint8_t stop_flag;    // 2bit
  uint8_t custom_flag;  // 1bit
  uint8_t reserved;     // 3bit (redundancy bits)
};

// ===============================
// 16bit 패킷 생성 (pack)
// ===============================
uint16_t packData(uint8_t speed_type, uint8_t speed_val,
                  uint8_t accel_type, uint8_t accel_val,
                  uint8_t stop_flag, uint8_t custom_flag,
                  uint8_t reserved)
{
  uint16_t packet = 0;

  packet |= (uint16_t)(speed_type  & 0x03) << 14;
  packet |= (uint16_t)(speed_val   & 0x07) << 11;
  packet |= (uint16_t)(accel_type  & 0x03) << 9;
  packet |= (uint16_t)(accel_val   & 0x07) << 6;
  packet |= (uint16_t)(stop_flag   & 0x03) << 4;
  packet |= (uint16_t)(custom_flag & 0x01) << 3;
  packet |= (uint16_t)(reserved    & 0x07);

  return packet;
}

// ===============================
// 16bit 패킷 해석 (unpack)
// ===============================
void unpackData(uint16_t packet, PacketData &data)
{
  data.speed_type  = (packet >> 14) & 0x03;
  data.speed_val   = (packet >> 11) & 0x07;
  data.accel_type  = (packet >> 9)  & 0x03;
  data.accel_val   = (packet >> 6)  & 0x07;
  data.stop_flag   = (packet >> 4)  & 0x03;
  data.custom_flag = (packet >> 3)  & 0x01;
  data.reserved    = (packet >> 0)  & 0x07;
}

#endif // PACKET_PROTOCOL_H
