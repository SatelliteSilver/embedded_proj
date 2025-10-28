
//You have to download UartQueue.h, UartQueue.cpp, PacketProtocol.h in github arduino/library

#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

// SoftwareSerial for Bluetooth
SoftwareSerial btSerial(2, 3);  // RX=2, TX=3

UartQueue rxQueue;   // 블루투스 수신 버퍼

// --디버깅용 보드에 올릴시 주석처리!!!!!!---///
void printBinary32(uint32_t value) {
  for (int i = 31; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" ");
  }
}
//---------------------------------------///

void setup() {
  Serial.begin(115200);   // 하드웨어 UART (D0/D1) → 아두이노B 연결
  btSerial.begin(115200); // 블루투스 모듈 (HC-05) 연결
  Serial.println("Bluetooth → UART Bridge Ready");
}

void loop() {
  // 1️⃣ 블루투스 데이터 수신
  while (btSerial.available()) {
    uint8_t b = btSerial.read();
    rxQueue.enqueue(b);
  }

  // 2️⃣ 4바이트(32bit) 완성 시 처리
  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4;
    rxQueue.dequeue(b1);
    rxQueue.dequeue(b2);
    rxQueue.dequeue(b3);
    rxQueue.dequeue(b4);

    uint32_t packet = ((uint32_t)b1 << 24) |
                      ((uint32_t)b2 << 16) |
                      ((uint32_t)b3 << 8) |
                      b4;

    // send UART(D0/D1) → board A
    Serial.write(b1);
    Delay(500);
    Serial.write(b2);
    Delay(500);
    Serial.write(b3);
    Delay(500);
    Serial.write(b4);

    // 🧠 시리얼 모니터 출력 (디코딩용) 
    uint8_t speed_type = (packet >> 30) & 0x03;
    uint8_t speed_val  = (packet >> 22) & 0xFF;
    uint8_t angle_type = (packet >> 20) & 0x03;
    uint8_t angle_val  = (packet >> 12) & 0xFF;
    uint8_t mode_type  = (packet >> 10) & 0x03;
    uint8_t mode_val   = (packet >> 2)  & 0xFF;
    uint8_t reserved   = packet & 0x03;

    // Serial.print("\n[BT→UART] HEX=0x");
    // Serial.print(packet, HEX);
    // Serial.print("  BIN=");
    // printBinary32(packet); // 디버깅할때 사용!!!!!! => 32bit 잘 나오는지 0100_1101.... 보드에 올릴때는 주석처리!
    // Serial.println();

    // Serial.print("  speed_val="); Serial.print(speed_val);
    // Serial.print("  angle_val="); Serial.print(angle_val);
    // Serial.print("  mode_val="); Serial.print(mode_val);
    // Serial.print("  reserved="); Serial.println(reserved);
  }
}
