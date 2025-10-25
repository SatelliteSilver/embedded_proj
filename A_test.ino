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




//------------32bit 패킷 기반 송수신 제어 코드 (UART + Bluetooth)-------------------//
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

// UART 통신 (아두이노 B와 연결)
SoftwareSerial uart(2, 3);  // RX=2, TX=3

// 블루투스 통신 (컴퓨터와 연결)
SoftwareSerial bluetooth(4, 5);  // RX=4, TX=5

// UART용 큐
UartQueue uartTxQueue;
UartQueue uartRxQueue;

// 블루투스용 큐
UartQueue btTxQueue;
UartQueue btRxQueue;

unsigned long lastSend = 0;


void setup() {
  Serial.begin(9600);
  uart.begin(9600);
  bluetooth.begin(9600);
  Serial.println("System Ready");
}

void loop() {
  // UART 처리
  while (uart.available()) {
    uint8_t b = uart.read();
    if (!uartRxQueue.enqueue(b)) {
      uartRxQueue.clear();
    }
  }

  while (uartRxQueue.hasCompletePacket()) {
    uint32_t packet;
    if (!uartRxQueue.dequeue32bit(packet)) {
      break;
    }

    if (((packet >> 30) & 0x03) != 0x01) continue;
    if (((packet >> 20) & 0x03) != 0x02) continue;
    if (((packet >> 10) & 0x03) != 0x03) continue;

    PacketData d;
    unpackData(packet, d);

    if (d.stop_flag == false) {
      btTxQueue.enqueue32bit(packet);
      continue;
    } 
    else if (d.stop_flag == true) {
      if (millis() - lastSend > 1000) {
        lastSend = millis();
        uint8_t speedVal = 100;
        bool accelSign = false;
        uint8_t accelVal = 50;
        bool stopFlag = true;
        uint16_t reserved = 0x1FF;
        uint32_t outPacket = packData(speedVal, accelSign, accelVal, stopFlag, reserved);
        uartTxQueue.enqueue32bit(outPacket);
      }
    }
  }

  // 블루투스 처리
  while (bluetooth.available()) {
    uint8_t b = bluetooth.read();
    if (!btRxQueue.enqueue(b)) {
      btRxQueue.clear();
    }
  }

  while (btRxQueue.hasCompletePacket()) {
    uint32_t packet;
    if (!btRxQueue.dequeue32bit(packet)) break;
    if (((packet >> 30) & 0x03) != 0x01) continue;
    if (((packet >> 20) & 0x03) != 0x02) continue;
    if (((packet >> 10) & 0x03) != 0x03) continue;
    uartTxQueue.enqueue32bit(packet);
  }

  // 전송 처리
  while (!btTxQueue.isEmpty()) {
    uint8_t out;
    btTxQueue.dequeue(out);
    bluetooth.write(out);
  }

  while (!uartTxQueue.isEmpty()) {
    uint8_t out;
    uartTxQueue.dequeue(out);
    uart.write(out);
  }
}
