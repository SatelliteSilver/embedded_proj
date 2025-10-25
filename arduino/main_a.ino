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

// 32bit 이진수 출력용
void printBinary32(uint32_t value) {
  for (int i = 31; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" ");
  }
}

void setup() {
  Serial.begin(9600);
  uart.begin(9600);
  bluetooth.begin(9600);
  Serial.println("UART + Bluetooth 32bit Packet TX/RX Control System");
  Serial.println("Packet Format: 01(v)8bit(속도)10(a)8bit(가속도)11(S)1bit + 9bit redundant");
  Serial.println("UART: Arduino B <-> Arduino A");
  Serial.println("Bluetooth: Computer <-> Arduino A");
}

void loop() {
  // ==================== UART 처리 (아두이노 B와 통신) ====================
  // UART로 들어온 바이트를 수신 큐에 적재
  while (uart.available()) {
    uint8_t b = uart.read();
    if (!uartRxQueue.enqueue(b)) {
      Serial.println("[UART-RX] Queue full! Dropping data.");
      uartRxQueue.clear(); // 큐가 가득 찬 경우 초기화
    }
  }

  // UART 큐에서 32bit 패킷이 완성되었는지 확인
  while (uartRxQueue.hasCompletePacket()) {
    uint32_t packet;
    if (!uartRxQueue.dequeue32bit(packet)) {
      break;
    }

    // 패킷 구조 검증 (고정 비트들 확인)
    if (((packet >> 30) & 0x03) != 0x01) {  // 비트 31-30: 01
      Serial.println("[UART-RX] Invalid packet header (01)!");
      continue;
    }
    if (((packet >> 20) & 0x03) != 0x02) {  // 비트 21-20: 10
      Serial.println("[UART-RX] Invalid packet header (10)!");
      continue;
    }
    if (((packet >> 10) & 0x03) != 0x03) {  // 비트 11-10: 11
      Serial.println("[UART-RX] Invalid packet header (11)!");
      continue;
    }

    PacketData d;
    unpackData(packet, d);

    Serial.print("[UART-RX] HEX=0x");
    Serial.print(packet, HEX);
    Serial.print("  BIN=");
    printBinary32(packet);
    Serial.println();

    // ---------------------------
    // stopFlag 비트로 모드 결정 (원본 패킷 기준)
    // ---------------------------
    if (d.stop_flag == false) {
      Serial.println("🟢 UART stopFlag=0 → RX 전용 모드");
      Serial.print("  Speed: "); Serial.print(d.speed_val);
      Serial.print(", Accel: "); Serial.print(d.accel_sign ? "-" : "+"); Serial.print(d.accel_val);
      Serial.print(", Reserved: 0x"); Serial.println(d.reserved, HEX);
      
      // UART 수신된 데이터를 블루투스로 전달 (원본 패킷)
      if (btTxQueue.enqueue32bit(packet)) {
        Serial.println("📤 UART -> Bluetooth forwarding");
      }
      // 수신만 처리하고 송신 금지
      continue;
    } 
    else if (d.stop_flag == true) {
      Serial.println("🔴 UART stopFlag=1 → TX 전용 모드");
      
      // TX만 수행 (새 패킷 생성 및 전송)
      if (millis() - lastSend > 1000) {
        lastSend = millis();

        uint8_t speedVal = 100;        // 속도값 예시
        bool accelSign = false;        // 가속도 부호 (false: +, true: -)
        uint8_t accelVal = 50;         // 가속도값 예시
        bool stopFlag = true;          // TX 모드 표시
        uint16_t reserved = 0x1FF;      // redundant bits (9bit 최대값)

        uint32_t outPacket = packData(speedVal, accelSign, accelVal, 
                                      stopFlag, reserved);

        // 32bit 패킷을 UART 큐에 추가
        if (uartTxQueue.enqueue32bit(outPacket)) {
          Serial.print("[UART-TX] HEX=0x");
          Serial.print(outPacket, HEX);
          Serial.print("  BIN=");
          printBinary32(outPacket);
          Serial.println();
          Serial.print("  Speed: "); Serial.print(speedVal);
          Serial.print(", Accel: "); Serial.print(accelSign ? "-" : "+"); Serial.print(accelVal);
          Serial.print(", Reserved: 0x"); Serial.println(reserved, HEX);
        } else {
          Serial.println("[UART-TX] Queue full, transmission failed!");
        }
      }
    }
  }

  // ==================== 블루투스 처리 (컴퓨터와 통신) ====================
  // 블루투스로 들어온 바이트를 수신 큐에 적재
  while (bluetooth.available()) {
    uint8_t b = bluetooth.read();
    if (!btRxQueue.enqueue(b)) {
      Serial.println("[BT-RX] Queue full! Dropping data.");
      btRxQueue.clear(); // 큐가 가득 찬 경우 초기화
    }
  }

  // 블루투스 큐에서 32bit 패킷이 완성되었는지 확인
  while (btRxQueue.hasCompletePacket()) {
    uint32_t packet;
    if (!btRxQueue.dequeue32bit(packet)) {
      break;
    }

    // 패킷 구조 검증 (고정 비트들 확인)
    if (((packet >> 30) & 0x03) != 0x01) {  // 비트 31-30: 01
      Serial.println("[BT-RX] Invalid packet header (01)!");
      continue;
    }
    if (((packet >> 20) & 0x03) != 0x02) {  // 비트 21-20: 10
      Serial.println("[BT-RX] Invalid packet header (10)!");
      continue;
    }
    if (((packet >> 10) & 0x03) != 0x03) {  // 비트 11-10: 11
      Serial.println("[BT-RX] Invalid packet header (11)!");
      continue;
    }

    PacketData d;
    unpackData(packet, d);

    Serial.print("[BT-RX] HEX=0x");
    Serial.print(packet, HEX);
    Serial.print("  BIN=");
    printBinary32(packet);
    Serial.println();

    // 블루투스 수신된 데이터를 UART로 전달
    if (uartTxQueue.enqueue32bit(packet)) {
      Serial.println("📤 Bluetooth -> UART forwarding");
      Serial.print("  Speed: "); Serial.print(d.speed_val);
      Serial.print(", Accel: "); Serial.print(d.accel_sign ? "-" : "+"); Serial.print(d.accel_val);
      Serial.print(", StopFlag: "); Serial.print(d.stop_flag ? "1" : "0");
      Serial.print(", Reserved: 0x"); Serial.println(d.reserved, HEX);
    } else {
      Serial.println("[UART-TX] Queue full, transmission failed!");
    }
  }

  // ==================== 블루투스 전송 처리 ====================
  // 블루투스 큐에서 데이터 전송
  while (!btTxQueue.isEmpty()) {
    uint8_t out;
    btTxQueue.dequeue(out);
    bluetooth.write(out);
  }

  // ==================== UART 전송 처리 ====================
  // UART 큐에서 데이터 전송
  while (!uartTxQueue.isEmpty()) {
    uint8_t out;
    uartTxQueue.dequeue(out);
    uart.write(out);
  }
}
