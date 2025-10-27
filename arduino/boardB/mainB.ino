
///-------------------------------------최종1!!!!!!!---------------------------------------------------------------////
//다음 A-B 송수신 +블투 버전//
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

// 블루투스 연결용 SoftwareSerial
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

    // 🟩 그대로 하드웨어 UART(D0/D1)로 전송 (→ Arduino B)
    Serial.write(b1);
    Serial.write(b2);
    Serial.write(b3);
    Serial.write(b4);

    // 🧠 시리얼 모니터 출력 (디코딩용) // 
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





//////////////////////////////////////////////////////////////////////////////////////////////////////





//현재 최종본//
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3);
//SoftwareSerial mySerial(6, 7); 

void setup() {
  Serial.begin(115200);
  BTSerial.begin(115200); 
  
}
//목표
//1015가 들어올 때 10, 15로 나눠서 처리할 방법?
//a==10, b==15 충족 시 속도 v=10 a=15로 송신
//문자열 커팅 구현

void loop() {
  if (BTSerial.available()) {
    char c = BTSerial.read();
    if(c == 1)
    Serial.print("v10a10s10");

    if(c == 5)
    Serial.print("v50a50s50");
    // char c = BTSerial.read();
    // Serial.print((int)c);
    delay(1000);
  }
  
 //Serial.println("");
  if (Serial.available()) {
    char c = Serial.read();
    BTSerial.write(c);
    //Serial.print((int)c);
    // Serial.println("v10a10s10");
  }
}

//------------------------------------------------------------------------//


// header
#include <SoftwareSerial.h>

// Serial Define
SoftwareSerial BTSerial(2, 3);  //TX2/  RX3
// "FOR DEBUGGING" 디버깅용: 6,7번을 시리얼 통신용으로 사용합니다.
//SoftwareSerial mySerial(6, 7);

void setup() {
  // pinMode(8, OUTPUT);    // HC-05 핀 제어
  // digitalWrite(8, HIGH); // AT 사용 용도

  // for debugging
  // mySerial.begin(9600);

  Serial.begin(9600);   
  BTSerial.begin(9600); //At+UART:변경속도,0,0 (115200으로 변경시 사용)
  Serial.println("PC1 <-> PC2 bluetooth ready");  // ready
}

void loop() {
  if (BTSerial.available()) {
    int data = BTSerial.read();
    Serial.write(data);
  }

  if (Serial.available()) {
    int data = Serial.read();
    BTSerial.write(data);
  }
}

//아스키코드 그대로 출력
// #include <SoftwareSerial.h>

// SoftwareSerial BTSerial(2, 3);

// void setup() {
//   Serial.begin(115200);
//   BTSerial.begin(115200); 
// }

// void loop() {
//   if (BTSerial.available()) {
//     char c = BTSerial.read();
//     Serial.print((int)c);
//   }
  
//   if (Serial.available()) {
//     char c = Serial.read();
//     BTSerial.write(c);
//     Serial.print((int)c);
//   }
// }

// ///-------------------------------------최종1!!!!!!!---------------------------------------------------------------////
// //다음 A-B 송수신 +블투 버전//
// #include <SoftwareSerial.h>
// #include "UartQueue.h"
// #include "PacketProtocol.h"

// // 블루투스 연결용 SoftwareSerial
// SoftwareSerial btSerial(2, 3);  // RX=2, TX=3

// UartQueue rxQueue;   // 블루투스 수신 버퍼

// void printBinary32(uint32_t value) {
//   for (int i = 31; i >= 0; i--) {
//     Serial.print((value >> i) & 1);
//     if (i % 4 == 0) Serial.print(" ");
//   }
// }

// void setup() {
//   Serial.begin(115200);   // 하드웨어 UART (D0/D1) → 아두이노B 연결
//   btSerial.begin(115200); // 블루투스 모듈 (HC-05) 연결
//   Serial.println("Bluetooth → UART Bridge Ready");
// }

// void loop() {
//   // 1️⃣ 블루투스 데이터 수신
//   while (btSerial.available()) {
//     uint8_t b = btSerial.read();
//     rxQueue.enqueue(b);
//   }

//   // 2️⃣ 4바이트(32bit) 완성 시 처리
//   while (rxQueue.getLength() >= 4) {
//     uint8_t b1, b2, b3, b4;
//     rxQueue.dequeue(b1);
//     rxQueue.dequeue(b2);
//     rxQueue.dequeue(b3);
//     rxQueue.dequeue(b4);

//     uint32_t packet = ((uint32_t)b1 << 24) |
//                       ((uint32_t)b2 << 16) |
//                       ((uint32_t)b3 << 8) |
//                       b4;

//     // 🟩 그대로 하드웨어 UART(D0/D1)로 전송 (→ Arduino B)
//     Serial.write(b1);
//     Serial.write(b2);
//     Serial.write(b3);
//     Serial.write(b4);

//     // 🧠 시리얼 모니터 출력 (디코딩용)
//     uint8_t speed_type = (packet >> 30) & 0x03;
//     uint8_t speed_val  = (packet >> 22) & 0xFF;
//     uint8_t angle_type = (packet >> 20) & 0x03;
//     uint8_t angle_val  = (packet >> 12) & 0xFF;
//     uint8_t mode_type  = (packet >> 10) & 0x03;
//     uint8_t mode_val   = (packet >> 2)  & 0xFF;
//     uint8_t reserved   = packet & 0x03;

//     // Serial.print("\n[BT→UART] HEX=0x");
//     // Serial.print(packet, HEX);
//     // Serial.print("  BIN=");
//     // printBinary32(packet);
//     // Serial.println();

//     // Serial.print("  speed_val="); Serial.print(speed_val);
//     // Serial.print("  angle_val="); Serial.print(angle_val);
//     // Serial.print("  mode_val="); Serial.print(mode_val);
//     // Serial.print("  reserved="); Serial.println(reserved);
//   }
// }
