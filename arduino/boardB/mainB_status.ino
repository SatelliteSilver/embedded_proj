
///-------------------------------------최종1!!!!!!!---------------------------------------------------------------////
//다음 A-B 송수신 +블투 버전//
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

// 블루투스 연결용 SoftwareSerial
SoftwareSerial btSerial(2, 3);  // RX=2, TX=3

UartQueue rxQueue;   // 블루투스 수신 버퍼

void printBinary32(uint32_t value) {
  for (int i = 31; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" ");
  }
}

void setup() {
  Serial.begin(115200);   // 하드웨어 UART (D0/D1) → 아두이노B 연결
  btSerial.begin(115200); // 블루투스 모듈 (HC-05) 연결
  Serial.println("Bluetooth → UART Bridge Ready"); // 보드에 올릴시 반드시 주석처리
}

void loop() {
  // 1️⃣ (기존 로직) PC -> CAR 방향 데이터 전달
  // 블루투스(PC)에서 데이터가 오면 큐에 쌓는다.
  while (btSerial.available()) {
    uint8_t b = btSerial.read();
    rxQueue.enqueue(b);
  }

  // 큐에 4바이트가 모이면 패킷을 조립해서 유선(CAR)으로 보낸다.
  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4;
    rxQueue.dequeue(b1);
    rxQueue.dequeue(b2);
    rxQueue.dequeue(b3);
    rxQueue.dequeue(b4);

    // 그대로 하드웨어 UART(D0/D1)로 전송 (→ CAR 아두이노) 
    Serial.write(b1);
    delay(200);
    Serial.write(b2);
    delay(200);
    Serial.write(b3);
    delay(200);
    Serial.write(b4);
  }


  // ▼▼▼ 여기에 추가 ▼▼▼
  // 2️⃣ (추가된 로직) CAR -> PC 방향 데이터 전달
  // 유선(CAR)에서 데이터가 오면 블루투스(PC)로 즉시 전달한다.
  while (Serial.available()) {
    uint8_t data_from_car = Serial.read();
    btSerial.write(data_from_car);
  }
}





