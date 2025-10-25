// -----------------송수신 가능 첫번째 모델--------------------//
// #include <SoftwareSerial.h>
// #include "UartQueue.h"
// #include "PacketProtocol.h"

// SoftwareSerial uart(2, 3);  // RX=2, TX=3
// UartQueue txQueue;
// UartQueue rxQueue;

// unsigned long lastSend = 0;

// // 16bit 이진수 출력용
// void printBinary16(uint16_t value) {
//   for (int i = 15; i >= 0; i--) {
//     Serial.print((value >> i) & 1);
//     if (i % 4 == 0) Serial.print(" ");
//   }
// }

// void setup() {
//   Serial.begin(9600);
//   uart.begin(9600);
//   Serial.println("UART Full-Duplex Node Ready");
// }

// void loop() {
//   // 1️⃣ 주기적 송신 (1초 간격)
//   if (millis() - lastSend > 1000) {
//     lastSend = millis();

//     // 패킷 생성
//     uint8_t speedType = 1;
//     uint8_t speedVal  = 0;
//     uint8_t accelType = 2;
//     uint8_t accelVal  = 0;
//     uint8_t stopFlag  = 3;
//     uint8_t customFlag= 1;
//     uint8_t reserved  = 0b101; // redundancy bits

//     uint16_t packet = packData(speedType, speedVal,
//                                accelType, accelVal,
//                                stopFlag, customFlag,
//                                reserved);

//     uint8_t hi = (packet >> 8) & 0xFF;
//     uint8_t lo = packet & 0xFF;

//     txQueue.enqueue(hi);
//     txQueue.enqueue(lo);

//     // 실제 전송
//     while (!txQueue.isEmpty()) {
//       uint8_t out;
//       txQueue.dequeue(out);
//       uart.write(out);
//     }

//     Serial.print("[TX] 0x");
//     Serial.print(packet, HEX);
//     Serial.print("  BIN=");
//     printBinary16(packet);
//     Serial.println();
//   }

//   // 2️⃣ 수신 처리
//   while (uart.available()) {
//     uint8_t b = uart.read();
//     rxQueue.enqueue(b);
//   }

//   while (rxQueue.getLength() >= 2) {
//     uint8_t hi, lo;
//     rxQueue.dequeue(hi);
//     rxQueue.dequeue(lo);

//     uint16_t packet = ((uint16_t)hi << 8) | lo;

//     // redundancy 검증
//     uint8_t redundancy = packet & 0x07;
//     if (redundancy != 0b101) {
//       Serial.println("[ERR] Redundancy check failed!");
//       continue;
//     }

//     PacketData d;
//     unpackData(packet, d);

//     Serial.print("[RX] HEX=0x");
//     Serial.print(packet, HEX);
//     Serial.print("  BIN=");
//     printBinary16(packet);
//     Serial.println();
//   }
// }




//------------stopflag값따라 송수신 제어 코드-------------------//
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

SoftwareSerial uart(2, 3);  // RX=2, TX=3
UartQueue txQueue;
UartQueue rxQueue;

unsigned long lastSend = 0;

// 16bit 이진수 출력용
void printBinary16(uint16_t value) {
  for (int i = 15; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" ");
  }
}

void setup() {
  Serial.begin(9600);
  uart.begin(9600);
  Serial.println("UART stopFlag-based TX/RX control (from packet bits)");
}

void loop() {
  // UART로 들어온 바이트를 수신 큐에 적재
  while (uart.available()) {
    uint8_t b = uart.read();
    rxQueue.enqueue(b);
  }

  // 큐에서 2바이트씩 꺼내서 패킷 복원
  while (rxQueue.getLength() >= 2) {
    uint8_t hi, lo;
    rxQueue.dequeue(hi);
    rxQueue.dequeue(lo);
    uint16_t packet = ((uint16_t)hi << 8) | lo;

    // redundancy (하위 3비트) 검사
    if ((packet & 0x07) != 0b101) {
      Serial.println("[RX] Redundancy check failed!");
      continue;
    }

    PacketData d;
    unpackData(packet, d);

    Serial.print("[RX] HEX=0x");
    Serial.print(packet, HEX);
    Serial.print("  BIN=");
    printBinary16(packet);
    Serial.println();

    // ---------------------------
    // stopFlag 비트로 모드 결정
    // ---------------------------
    if (d.stop_flag == 0) {
      Serial.println("🟢 stopFlag=0 → RX 전용 모드");
      // 수신만 처리하고 송신 금지
      // (이 보드는 지금 RX만 가능)
      continue;
    } 
    else if (d.stop_flag == 1) {
      Serial.println("🔴 stopFlag=1 → TX 전용 모드");
      
      // TX만 수행 (새 패킷 생성 및 전송)
      if (millis() - lastSend > 1000) {
        lastSend = millis();

        uint8_t speedType = 1;
        uint8_t speedVal  = 0;
        uint8_t accelType = 2;
        uint8_t accelVal  = 0;
        uint8_t stopFlag  = 1;   // 이번 전송 패킷에도 TX 상태 표시
        uint8_t customFlag= 1;
        uint8_t reserved  = 0b101;

        uint16_t outPacket = packData(speedType, speedVal,
                                      accelType, accelVal,
                                      stopFlag, customFlag,
                                      reserved);

        uint8_t outHi = (outPacket >> 8) & 0xFF;
        uint8_t outLo = outPacket & 0xFF;

        txQueue.enqueue(outHi);
        txQueue.enqueue(outLo);

        while (!txQueue.isEmpty()) {
          uint8_t out;
          txQueue.dequeue(out);
          uart.write(out);
        }

        Serial.print("[TX] HEX=0x");
        Serial.print(outPacket, HEX);
        Serial.print("  BIN=");
        printBinary16(outPacket);
        Serial.println();
      }
    }
  }
}
