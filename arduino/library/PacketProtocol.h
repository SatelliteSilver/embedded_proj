//
// PacketProtocol.h
// 32bit UART 데이터 패킷 정의 및 비트 해석 유틸
//

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <Arduino.h>

// 데이터 필드 구조 정의
struct PacketData {
  uint8_t speed_val;     // 8bit 속도값
  bool accel_sign;        // 1bit 가속도 부호 (0: +, 1: -)
  uint8_t accel_val;      // 7bit 가속도값
  bool stop_flag;         // 1bit stop_flag (S) - 1일 때만 TX
  uint16_t reserved;      // 9bit redundant bits
};

// ===============================
// 32bit 패킷 생성 (pack)
// ===============================
uint32_t packData(uint8_t speed_val, bool accel_sign, uint8_t accel_val, 
                  bool stop_flag, uint16_t reserved)
{
  uint32_t packet = 0;

  // 비트 31-30: 01 (고정)
  packet |= 0x40000000;
  
  // 비트 29-22: 속도값 (8bit)
  packet |= (uint32_t)(speed_val & 0xFF) << 22;
  
  // 비트 21-20: 10 (고정)
  packet |= 0x200000;
  
  // 비트 19: 부호 비트 (0: +, 1: -)
  packet |= (uint32_t)(accel_sign ? 1 : 0) << 19;
  
  // 비트 18-12: 가속도값 (7bit)
  packet |= (uint32_t)(accel_val & 0x7F) << 12;
  
  // 비트 11-10: 11 (고정)
  packet |= 0xC00;
  
  // 비트 9: stop_flag
  packet |= (uint32_t)(stop_flag ? 1 : 0) << 9;
  
  // 비트 8-0: redundant bits (9bit)
  packet |= (uint32_t)(reserved & 0x1FF);

  return packet;
}

// ===============================
// 32bit 패킷 해석 (unpack)
// ===============================
void unpackData(uint32_t packet, PacketData &data)
{
  // 비트 29-22: 속도값
  data.speed_val = (packet >> 22) & 0xFF;
  
  // 비트 19: 부호 비트
  data.accel_sign = (packet >> 19) & 0x01;
  
  // 비트 18-12: 가속도값
  data.accel_val = (packet >> 12) & 0x7F;
  
  // 비트 9: stop_flag
  data.stop_flag = (packet >> 9) & 0x01;
  
  // 비트 8-0: redundant bits
  data.reserved = packet & 0x1FF;
}

#endif // PACKET_PROTOCOL_H
