//
// PacketProtocol.h
// 32bit UART defining packData and bit utility unpack
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
// 32bit 패킷 생성 (pack)
// ===============================
uint32_t packData(uint8_t speed_type,  uint8_t speed_val,
                  uint8_t accel_type,  uint8_t accel_sign,
                  uint8_t accel_val,   uint8_t stop_flag,
                  uint8_t custom_flag, uint8_t reserved)
{
  // 1. 패킷 변수는 32비트
  uint32_t packet = 0;

  // speed_type (2 bits) : bits 31-30 
  packet |= (uint32_t)(speed_type & 0x03) << 30; // 30bits shift
  // speed_val (8 bits) : bits 29-22 (0-255의 unsigned 값)
  packet |= (uint32_t)(speed_val & 0xFF) << 22; // 22bits shift
  // accel_type (2 bits) : bits 21-20
  packet |= (uint32_t)(accel_type & 0x03) << 20; // 20bits shift
  // accel_sign (1 bit) : bit 19
  packet |= (uint32_t)(accel_sign & 0x01) << 19; // 19bits shift
  // accel_val (7 bits) : bits 18-12 (0-127의 절대값) 7비트 마스크(0x7F) 사용
  packet |= (uint32_t)(accel_val & 0x7F) << 12; // 12bits shift
  // stop_flag (2 bits) : bits 11-10
  packet |= (uint32_t)(stop_flag & 0x03) << 10;
  // custom_flag (1 bit) : bit 9
  packet |= (uint32_t)(custom_flag & 0x01) << 9;
  // reserved (9 bits) : bits 8-0 (시프트 없음, 9비트 마스크 0x1FF)
  packet |= (uint32_t)(reserved & 0x1FF);

  return packet;
}

// ===============================
// 32bit 패킷 해석 (unpack)
// ===============================
void unpackData(uint32_t packet, PacketData &data)
{
  // 32비트 구조에 맞게 시프트(>>) 및 마스크(&) 값 변경

  // speed_type (2 bits) : bits 31-30
  data.speed_type   = (packet >> 30) & 0x03;
  // speed_val (8 bits) : bits 29-22
  data.speed_val    = (packet >> 22) & 0xFF; // 8비트 마스크
  // accel_type (2 bits) : bits 21-20
  data.accel_type   = (packet >> 20) & 0x03;
  // accel_sign (1 bit) : bit 19 (새로 추가)
  data.accel_sign   = (packet >> 19) & 0x01;
  // accel_val (7 bits) : bits 18-12
  data.accel_val    = (packet >> 12) & 0x7F; // 7비트 마스크
  // stop_flag (2 bits) : bits 11-10
  data.stop_flag    = (packet >> 10) & 0x03;
  // custom_flag (1 bit) : bit 9
  data.custom_flag  = (packet >> 9)  & 0x01;
  // reserved (9 bits) : bits 8-0
  data.reserved     = (packet >> 0)  & 0x1FF; // 9비트 마스크
}


#endif // PACKET_PROTOCOL_H
