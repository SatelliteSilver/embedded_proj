// ================================================================
// BT↔UART 브리지 + 초음파(HC‑SR04) 통합 (Arduino UNO, ATmega328P)
//   - HC‑05 블루투스(SoftwareSerial 2,3)
//   - 보드A와 하드웨어 UART(Serial, D0/D1)
//   - 32bit 패킷(4바이트) 브리지 + 거리(cm) 주기 전송
//   - UartQueue.h / PacketProtocol.h 필요 (사용자 깃허브 라이브러리) B 임(REGISTER X + 초음파)
//   - 핀 0,1,2,3 이미 사용 → 초음파는 6,7 권장
// ================================================================
#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"
// ---------------- 핀 설정 ----------------
// BT: HC‑05
static const uint8_tPIN_BT_RX = 2;  // Arduino RX <= BT TX
static const uint8_tPIN_BT_TX = 3;  // Arduino TX => BT RX
// 초음파: HC‑SR04 (0,1,2,3은 이미 사용 중이므로 6,7 사용)
static const uint8_tPIN_TRIG  = 6;  // PD6
static const uint8_tPIN_ECHO  = 7;  // PD7
// ---------------- 시리얼 ----------------
SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);
UartQueue     rxQueue;                // BT 수신 큐(바이트)
// 디버그: 32bit 이진수 프린트 (보드에 올릴 때는 비활성 권장)
// static void printBinary32(uint32_t value){
//   for (int i = 31; i >= 0; i--) {
//     Serial.print((value >> i) & 1);
//     if (i % 4 == 0) Serial.print(" ");
//   }
// }
// ---------------- 초음파 유틸 ----------------
static void ultrasonicInit(){
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
}
// HC‑SR04 거리 측정 (최대 160cm, 그 이상은 0 반환)
static int readDistanceCm(){
  // 10us 트리거 펄스
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  // ECHO High 펄스폭 읽기 (타임아웃 ~ 9.5ms)
  unsigned longdur = pulseIn(PIN_ECHO, HIGH, 9500UL);
  if (dur == 0) return 0;// 타임아웃
  // 거리 변환: cm ≈ dur(µs) / 58 (공식)
  intcm = (int)(dur / 58UL);
  if (cm > 160) return 0;// 요구사항: 160cm 초과 무효
  returncm;
}
// ---------------- 설정 ----------------
// 거리 전송 주기(ms)
static const unsigned longDIST_TX_INTERVAL_MS = 100;// 10Hz 정도
void setup(){
  Serial.begin(115200);    // 보드A 측 하드웨어 UART (D0/D1)
  btSerial.begin(115200);  // HC‑05 (D2/D3)
  ultrasonicInit();
  Serial.println("Bluetooth → UART Bridge + Ultrasonic Ready");
}
void loop() {
  // 1) 블루투스 수신 -> 큐 적재
  while (btSerial.available()) {
    rxQueue.enqueue((uint8_t)btSerial.read());
  }

  // 2) 4바이트(32bit) 패킷 완성 시: 그대로 보드A(Serial)로 바이너리 전달
  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4;
    rxQueue.dequeue(b1);
    rxQueue.dequeue(b2);
    rxQueue.dequeue(b3);
    rxQueue.dequeue(b4);

    // (디버깅 필요하면 packet 조립)
    // uint32_t packet = ((uint32_t)b1 << 24) |
    //                   ((uint32_t)b2 << 16) |
    //                   ((uint32_t)b3 << 8)  |
    //                   b4;

    // Board A(유선)로 그대로 전달 (지연 최소화)
    Serial.write(b1); delay(1);
    Serial.write(b2); delay(1);
    Serial.write(b3); delay(1);
    Serial.write(b4); // 마지막 바이트 뒤에는 delay 생략 가능
  }

  // 3) 주기적으로 거리(cm) 값을 전송
  static unsigned long lastDistMs = 0;
  unsigned long now = millis();
  if (now - lastDistMs >= DIST_TX_INTERVAL_MS) {
    lastDistMs = now;
    int cm = readDistanceCm();

    // ▼ 원하는 방향만 활성화하세요 ▼
    // (A) 보드A(유선)로 보내려면:
    // Serial.print("DIST,"); Serial.println(cm);

    // (B) 블루투스(PC)로 보내려면:
    btSerial.print("DIST,"); btSerial.println(cm);
  }

  // 4) CAR → PC 릴레이 (Board A → Bluetooth)
  // 유선(Serial)에 들어오는 모든 바이트를 블루투스(btSerial)로 전달
  while (Serial.available()) {
    uint8_t data_from_car = (uint8_t)Serial.read();
    btSerial.write(data_from_car);
  }
}// mainB_status 참고


