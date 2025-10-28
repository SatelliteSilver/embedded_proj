///board B 레지스터 코드!!!!!!!!!!!!!!!!!!/////
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

SoftwareSerial btSerial(2, 3);  // RX=2, TX=3 (SoftwareSerial)
UartQueue rxQueue;

void setup() {
  // 하드웨어 UART 직접 초기화 (D0/D1)
  UBRR0H = 0;
  UBRR0L = 16;                    // 115200 bps @ 16MHz
  UCSR0A = (1 << U2X0);           // 2배속 모드
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);                 // 수신/송신 활성화
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               // 8N1

  btSerial.begin(115200);         // 블루투스 시리얼
}

void loop() {
  // BT → 큐 적재
  while (btSerial.available()) {
    uint8_t b = btSerial.read();
    rxQueue.enqueue(b);
  }

  // 4바이트(32bit) 모이면 A 보드로 전송
  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4;
    rxQueue.dequeue(b1);
    rxQueue.dequeue(b2);
    rxQueue.dequeue(b3);
    rxQueue.dequeue(b4);

    // 필요 시 여기에서 packet을 파싱해도 됨
    // uint32_t packet = ((uint32_t)b1 << 24) |
    //                   ((uint32_t)b2 << 16) |
    //                   ((uint32_t)b3 << 8)  |
    //                   b4;

    // 하드웨어 UART 레지스터 직접 송신 (연속 전송)
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = b1;
    delay(200);
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = b2;
    delay(200);
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = b3;
    delay(200);
    while (!(UCSR0A & (1 << UDRE0))); UDR0 = b4;
  }
}
