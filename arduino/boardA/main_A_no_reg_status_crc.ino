#include <AltSoftSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

// ===== CRC-8 (0x07) for upper 24 bits (bits 31..8) =====
static uint8_t crc8_24(uint32_t upper24)
{
  uint8_t crc = 0x00;
  for (int i = 2; i >= 0; --i) {
    uint8_t b = (upper24 >> (i*8)) & 0xFF;
    crc ^= b;
    for (uint8_t k = 0; k < 8; ++k) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else            crc <<= 1;
    }
  }
  return crc;
}

// ===== 여기 전역 변수들만 모아두기 =====
const int DBG_LED_PIN = 7;
static uint8_t  g_tx_seqbit  = 0;   // ✅ STATUS 보낼 때 토글
static uint16_t crc_err_cnt  = 0;   // CRC 깨진 거 몇 번인지

AltSoftSerial imuSerial;
#define IMU_SERIAL imuSerial
UartQueue rxQueue;                  // ✅ 여기서만 한 번 선언


void blinkOnce() {
  digitalWrite(DBG_LED_PIN, HIGH);
  delay(60);
  digitalWrite(DBG_LED_PIN, LOW);
  delay(40);
}

void blinkTwice() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(DBG_LED_PIN, HIGH);
    delay(60);
    digitalWrite(DBG_LED_PIN, LOW);
    delay(40);
  }
}

void blinkThree() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(DBG_LED_PIN, HIGH);
    delay(50);
    digitalWrite(DBG_LED_PIN, LOW);
    delay(40);
  }
}





// ===== 핀 매핑 =====
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 11
#define IN3 12
#define IN4 13

// ===== 비트 마스크 (UNO) =====
#define IN1_BIT _BV(PD4)  // D4
#define IN2_BIT _BV(PD5)  // D5
#define IN3_BIT _BV(PB4)  // D12
#define IN4_BIT _BV(PB5)  // D13

// ===== 전역 구조체 =====
struct SAngle { float Roll, Pitch, Yaw; };
struct SAcc   { float X, Y, Z; };
struct SGyro  { float X, Y, Z; };
SAngle stcAngle; SAcc stcAcc; SGyro stcGyro;
float Axyz[3] = {0}, Gxyz[3] = {0};

// ===== 헤딩/상태 =====
float currentHeading = 0.0f, initialHeading = 0.0f, targetAngle = 0.0f;
int   targetSpeed = 0;
bool  newCommandReceived = false;
float yawZeroOffset = 0.0f, lastOutYaw = 0.0f;

enum RobotState { IDLE, ROTATING, MOVING };
RobotState currentState = IDLE;
unsigned long moveStartTime = 0;

// ===== PID =====
float Kp = 4.0f, Ki = 0.0f, Kd = 0.1f;

// ===== 모터 파라미터 =====
const int MIN_PWM = 140;
// 회전 시에만 적용되는 최소 PWM (직진과 분리 조정)
const int MIN_ROTATE_PWM = 90;
const int MAX_PWM = 250;
const int MAX_MOVE_PWM = 255;
const int INITIAL_MOVE_SPEED = 250;
const int INITIAL_MOVE_DURATION = 10;
const unsigned long MOVE_DURATION = 2000;

// ===== UART & 디버그 (Source 1) =====
#define UART_BAUD_RATE 9600
#define DEBUG 0
unsigned long lastDbgMs = 0;
const unsigned long DBG_PERIOD_MS = 200;
uint32_t accCnt = 0, gyrCnt = 0, angCnt = 0, lastCountMs = 0;

// ===== 유틸 =====
static inline float wrap360(float a){ while(a<0)a+=360; while(a>=360)a-=360; return a; }
static inline float angleDiff(float t,float c){ float d=t-c; while(d>180)d-=360; while(d<-180)d+=360; return d; }
const float ANGLE_DEADBAND = 3.0f;
bool  moving = false;
const float LPF_ALPHA = 0.25f;

void sendCompletionStatus() {
    uint32_t statusPacket = (1UL << 9);

    g_tx_seqbit ^= 1;  // ← 이제 전역에 있어서 OK
    statusPacket |= (uint32_t)(g_tx_seqbit & 0x1) << 8;

    uint8_t crc = crc8_24(statusPacket >> 8);
    statusPacket |= crc;

    uint8_t bytes[4];
    bytes[0] = (statusPacket >> 24) & 0xFF;
    bytes[1] = (statusPacket >> 16) & 0xFF;
    bytes[2] = (statusPacket >> 8)  & 0xFF;
    bytes[3] = statusPacket & 0xFF;

    Serial.write(bytes, 4);
    Serial.flush();

    blinkTwice();
}


void dbgPrintStateLine(const char* tag, float err) {
  if (!DEBUG) return;
  Serial.print(tag); Serial.print(" | STATE=");
  Serial.print((int)currentState);      // 0:IDLE, 1:ROTATING, 2:MOVING
  Serial.print(" v=");  Serial.print(targetSpeed);
  Serial.print(" hdg="); Serial.print(currentHeading, 1);
  Serial.print(" tgt="); Serial.print(targetAngle, 1);
  Serial.print(" err=");
  Serial.println(err, 1);
}

// ===== 모터 제어 (Timer2 PWM: D3=OC2B, D11=OC2A) (Source 2) =====
// (Source 1의 analogWrite/digitalWrite 대신 이 함수들이 사용됨)
void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  int pwmValue = abs(constrain(speed, -255, 255));

  if (EN_pin == ENA) { // 왼쪽
    if (speed > 0)      { PORTD |= IN1_BIT; PORTD &= ~IN2_BIT; } // D4=H, D5=L
    else if (speed < 0) { PORTD &= ~IN1_BIT; PORTD |= IN2_BIT; } // D4=L, D5=H
    else                { PORTD &= ~(IN1_BIT | IN2_BIT); }      // D4=L, D5=L
    OCR2B = pwmValue; // D3 PWM
  } else if (EN_pin == ENB) { // 오른쪽
    if (speed > 0)      { PORTB |= IN3_BIT; PORTB &= ~IN4_BIT; } // D12=H, D13=L
    else if (speed < 0) { PORTB &= ~IN3_BIT; PORTB |= IN4_BIT; } // D12=L, D13=H
    else                { PORTB &= ~(IN3_BIT | IN4_BIT); }      // D12=L, D13=L
    OCR2A = pwmValue; // D11 PWM
  }
}

void setMotorSpeedDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  moving = (abs(leftSpeed) >= MIN_PWM || abs(rightSpeed) >= MIN_PWM);

  if (!moving) {
    driveOneMotor(ENA, IN1, IN2, 0);
    driveOneMotor(ENB, IN3, IN4, 0);
    return;
  }
  if (abs(leftSpeed)  < MIN_PWM && leftSpeed  != 0) leftSpeed  = (leftSpeed  > 0) ? MIN_PWM : -MIN_PWM;
  if (abs(rightSpeed) < MIN_PWM && rightSpeed != 0) rightSpeed = (rightSpeed > 0) ? MIN_PWM : -MIN_PWM;

  driveOneMotor(ENA, IN1, IN2, leftSpeed);
  driveOneMotor(ENB, IN3, IN4, rightSpeed);
}

// 회전 전용: 원하는 최소 PWM을 전달하여 별도 조정 가능
void setMotorSpeedDifferentialWithMin(int leftSpeed, int rightSpeed, int minPwm) {
  leftSpeed  = constrain(leftSpeed,  -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  moving = (abs(leftSpeed) >= minPwm || abs(rightSpeed) >= minPwm);

  if (!moving) {
    driveOneMotor(ENA, IN1, IN2, 0);
    driveOneMotor(ENB, IN3, IN4, 0);
    return;
  }
  if (abs(leftSpeed)  < minPwm && leftSpeed  != 0) leftSpeed  = (leftSpeed  > 0) ? minPwm : -minPwm;
  if (abs(rightSpeed) < minPwm && rightSpeed != 0) rightSpeed = (rightSpeed > 0) ? minPwm : -minPwm;

  driveOneMotor(ENA, IN1, IN2, leftSpeed);
  driveOneMotor(ENB, IN3, IN4, rightSpeed);
}

void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  int corrected_left = -speed; // 왼쪽 모터 속도
  int corrected_right = -speed*0.8; // 예시: 오른쪽 모터 속도를 5% 줄임

  moving = (abs(corrected_left) >= MIN_PWM || abs(corrected_right) >= MIN_PWM);
  //int corrected = -speed; // 필요시 +speed로 바꾸세요
  //moving = (abs(corrected) >= MIN_PWM);
  driveOneMotor(ENA, IN1, IN2, corrected_left);
  driveOneMotor(ENB, IN3, IN4, corrected_right);
}

// ===== WT901 (Source 1 & 2) =====
const unsigned long WT901_BAUD = 9600;
uint8_t imuFrame[11]; uint8_t imuIdx = 0;

inline bool wt901CheckSumOK(const uint8_t* f){
  uint8_t s=0; for(int i=0;i<10;++i) s+=f[i]; return (s==f[10]);
}
void imuSetup(){ IMU_SERIAL.begin(WT901_BAUD); }

// [추가] WT901 파싱 파트 (from Source 1)
void imuPollWT901() {
  while (IMU_SERIAL.available()) {
    uint8_t c = (uint8_t)IMU_SERIAL.read();
    if (imuIdx == 0) {
      if (c != 0x55) continue;
      imuFrame[imuIdx++] = c;
      continue;
    }

    imuFrame[imuIdx++] = c;
    if (imuIdx == 2) {
      if (imuFrame[1] != 0x51 && imuFrame[1] != 0x52 && imuFrame[1] != 0x53) {
        imuIdx = (c == 0x55) ? 1 : 0;
      }
    }

    if (imuIdx == 11) {
      imuIdx = 0;
      if (!wt901CheckSumOK(imuFrame)) continue;

      auto s16 = [&](int lo, int hi) -> int16_t {
        return (int16_t)(((uint16_t)imuFrame[hi] << 8) | (uint16_t)imuFrame[lo]);
      };

      if (imuFrame[1] == 0x51) { // 가속도
        stcAcc.X = (float)s16(2, 3) / 32768.0f * 16.0f;
        stcAcc.Y = (float)s16(4, 5) / 32768.0f * 16.0f;
        stcAcc.Z = (float)s16(6, 7) / 32768.0f * 16.0f;
        Axyz[0] = stcAcc.X; Axyz[1] = stcAcc.Y; Axyz[2] = stcAcc.Z;
        accCnt++;
      } else if (imuFrame[1] == 0x52) { // 각속도
        stcGyro.X = (float)s16(2, 3) / 32768.0f * 2000.0f;
        stcGyro.Y = (float)s16(4, 5) / 32768.0f * 2000.0f;
        stcGyro.Z = (float)s16(6, 7) / 32768.0f * 2000.0f;
        Gxyz[0] = stcGyro.X; Gxyz[1] = stcGyro.Y; Gxyz[2] = stcGyro.Z;
        gyrCnt++;
      } else if (imuFrame[1] == 0x53) { // 각도
        stcAngle.Roll  = (float)s16(2, 3) / 32768.0f * 180.0f;
        stcAngle.Pitch = (float)s16(4, 5) / 32768.0f * 180.0f;
        stcAngle.Yaw   = (float)s16(6, 7) / 32768.0f * 180.0f;
        angCnt++;
      }
    } // end if (imuIdx == 11)
  } // end while (IMU_SERIAL.available())
}

// [추가] 기존 함수명 유지 (from Source 1)
void imuUpdate() {
  imuPollWT901();
}

// [추가] WT901의 Yaw를 사용해 헤딩 산출 (from Source 1)
void updateHeadingFused() {
  static bool init = true;
  static float outYaw = 0.0f; 

  float rawYaw360 = wrap360(stcAngle.Yaw < 0 ? (stcAngle.Yaw + 360.0f) : stcAngle.Yaw);
  float measured = rawYaw360; 

  if (init) {
    outYaw = measured;
    init = false;
  } else {
    float d = angleDiff(measured, outYaw);
    outYaw = wrap360(outYaw + LPF_ALPHA * d);
  }

  lastOutYaw = outYaw;
  currentHeading = wrap360(outYaw - yawZeroOffset);
}

/*
// [추가] ===== PID 회전 ===== (from Source 1)
bool rotateToAbsAngle(float targetAbsAngle) {
  float error = 0.0f;
  float integral = 0.0f, lastError = 0.0f, derivative = 0.0f;
  int correction = 0;

  unsigned long lastTime = micros(), startTime = millis();
  unsigned long lastPrintMs = millis();
  while (true) {
    if (millis() - startTime > 15000UL) {
      setMotorSpeedDifferential(0, 0);
      return false;
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01f;

    imuUpdate();
    updateHeadingFused();
    error = angleDiff(targetAbsAngle, currentHeading);

    if (DEBUG && millis() - lastPrintMs >= 100) {
      lastPrintMs = millis();
      Serial.print("[ROT] Target="); Serial.print(targetAbsAngle, 1);
      Serial.print(" Current="); Serial.print(currentHeading, 1);
      Serial.print(" Error="); Serial.print(error, 1);
      Serial.print(" Correction="); Serial.println(correction);
    }

    if (abs(error) <= ANGLE_DEADBAND) {
      setMotorSpeedDifferential(0, 0);
      return true;
    }

    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = (int)round(Kp * error + Ki * integral + Kd * derivative);

    if (abs(correction) < MIN_ROTATE_PWM) {
      correction = (correction >= 0) ? MIN_ROTATE_PWM : -MIN_ROTATE_PWM;
    }
    correction = constrain(correction, -MAX_PWM, MAX_PWM);

    int leftSpeed, rightSpeed;
    if (error > 0) {
      leftSpeed = abs(correction);
      rightSpeed = -abs(correction);
    } else {
      leftSpeed = -abs(correction);
      rightSpeed = abs(correction);
    }

    if (DEBUG) {
        Serial.print(" L="); Serial.print(leftSpeed);
        Serial.print(" R="); Serial.println(rightSpeed);
    }
    
    setMotorSpeedDifferentialWithMin(leftSpeed, rightSpeed, MIN_ROTATE_PWM);
    delay(10);
  } // end while
}*/

bool rotateToAbsAngle(float targetAbsAngle) {
  float error = 0.0f;
  float integral = 0.0f, lastError = 0.0f, derivative = 0.0f;
  int correction = 0;

  unsigned long lastTime = micros(), startTime = millis();
  unsigned long lastPrintMs = millis();
  while (true) {
    if (millis() - startTime > 15000UL) {
      setMotorSpeedDifferential(0, 0);
      return false;
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01f;

    // ① IMU는 계속 읽어오고
    imuUpdate();

    // ② 회전 중에는 "필터 안 거친" 즉시 yaw 를 쓴다
    float rawYaw = stcAngle.Yaw;
    if (rawYaw < 0) rawYaw += 360.0f;     // 0~360도로
    float heading_now = wrap360(rawYaw);  // 바로 현재 헤딩

    error = angleDiff(targetAbsAngle, heading_now);

    // 디버그 있으면 찍기
    if (DEBUG && millis() - lastPrintMs >= 100) {
      lastPrintMs = millis();
      Serial.print("[ROT] tgt="); Serial.print(targetAbsAngle, 1);
      Serial.print(" now=");      Serial.print(heading_now, 1);
      Serial.print(" err=");      Serial.println(error, 1);
    }

    // ③ 충분히 가까우면 끝
    if (abs(error) <= ANGLE_DEADBAND) {
      setMotorSpeedDifferential(0, 0);
      return true;
    }

    // ④ PID 그대로
    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = (int)round(Kp * error + Ki * integral + Kd * derivative);

    // ⑤ 타깃에 근접했을 때는 세게 안 돌게 살짝 줄여주기
    int minPwm = MIN_ROTATE_PWM;  // 원래 100이었지
    if (abs(error) < 25.0f) {     // 25도 안 쪽으로 들어오면
      correction = correction / 2;
      if (correction == 0) correction = (error > 0) ? 40 : -40;
      minPwm = 50;                // 너무 세게 안 돌게
    }

    if (abs(correction) < minPwm) {
      correction = (correction >= 0) ? minPwm : -minPwm;
    }
    correction = constrain(correction, -MAX_PWM, MAX_PWM);

    int leftSpeed, rightSpeed;
    if (error > 0) {
      leftSpeed  =  abs(correction);
      rightSpeed = -abs(correction);
    } else {
      leftSpeed  = -abs(correction);
      rightSpeed =  abs(correction);
    }

    setMotorSpeedDifferentialWithMin(leftSpeed, rightSpeed, minPwm);
    delay(10);
  }
}


// [추가] 비트 리버스 함수 (from Source 1)
uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// [추가] 디버깅용 32비트 바이너리 출력 함수 (from Source 1)
void printBinary32(uint32_t value) {
  for (int i = 31; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i % 4 == 0) Serial.print(" ");
  }
}

// ===== Arduino 기본 Setup (Source 1 로직 + Source 2 레지스터) =====
void setup() {
  pinMode(DBG_LED_PIN, OUTPUT);
  digitalWrite(DBG_LED_PIN, LOW);

  // 1. 패킷 수신용 + 디버그 출력용 (D0, D1) UART 초기화 (Source 1)
  Serial.begin(UART_BAUD_RATE);
  
  
  // 2. WT901 센서용 (D8, D9) AltSoftSerial 초기화 (Source 1)
  imuSetup();

  // 3. 모터 핀 출력 설정 (Source 2 레지스터 방식)
  // DDR (Data Direction Register) 설정: 1=OUTPUT
  DDRD |= IN1_BIT | IN2_BIT | _BV(PD3); // D4, D5, D3(ENA) 출력
  DDRB |= IN3_BIT | IN4_BIT | _BV(PB3); // D12, D13, D11(ENB) 출력

  // 4. 포트 초기값 LOW (Source 2)
  PORTD &= ~(IN1_BIT | IN2_BIT); // D4, D5 LOW
  PORTB &= ~(IN3_BIT | IN4_BIT); // D12, D13 LOW

  // 5. Timer2: Fast PWM, prescaler 64 로 변경!
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1<<WGM21) | (1<<WGM20);    // Fast PWM (Mode 3, TOP=0xFF)
  TCCR2A |= (1<<COM2A1) | (1<<COM2B1);  // Non-inverting on OC2A(D11), OC2B(D3)

  // ★ prescaler = 64 로 설정 (CS22=1, CS21=0, CS20=0)
  TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20)); // 기존 비트 클리어
  TCCR2B |=  (1<<CS22);                       // CS22 비트만 1로 설정

  OCR2A = 0; // D11 PWM 초기화
  OCR2B = 0; // D3  PWM 초기화

  // 6. 시작 시 모터 워밍업 (Source 1)
  moveAtSpeed(INITIAL_MOVE_SPEED);
  delay(INITIAL_MOVE_DURATION);
  moveAtSpeed(0);
  delay(500); // 잠시 대기

  // 7. 초기 헤딩 설정 (IMU 안정화 대기) (Source 1)
  unsigned long t0 = millis();
  uint16_t angSamples = 0;
  float lastYawSample = 9999.0f;

  if (DEBUG) Serial.print("Waiting for stable IMU angle data...");

  while (millis() - t0 < 2000) {  // 최대 2초 대기
    imuUpdate(); 
    updateHeadingFused(); 

    if (stcAngle.Yaw != lastYawSample) {
      lastYawSample = stcAngle.Yaw;
      angSamples++;
      if (DEBUG) Serial.print("."); 
      if (angSamples >= 5) break; // 안정화 완료
    }
    delay(10);
  }
  if (DEBUG) Serial.println(" OK."); 

  // 8. 현재 방향을 0도로 설정 (영점) (Source 1)
  yawZeroOffset = lastOutYaw;
  updateHeadingFused();
  initialHeading = currentHeading;
  targetAngle    = initialHeading;

  // 9. 초기화 완료 후 정지 및 상태 설정 (Source 1)
  setMotorSpeedDifferential(0, 0); 
  moveAtSpeed(0);                  
  currentState = IDLE;
  newCommandReceived = false;
  targetSpeed = 0;

  if (DEBUG) Serial.println("Setup complete. Robot stopped. Waiting for command...");
}


// ===== Arduino 기본 Loop (Source 1 로직) =====
void loop() {

  // 1️⃣ 수신 데이터 버퍼링 (읽을 때 비트 리버스 적용)
  while (Serial.available()) {
    // ▼▼▼ 여기서 첫 번째 reverseBits 적용 ▼▼▼
    uint8_t b = reverseBits(Serial.read());
    rxQueue.enqueue(b);
  }

  // 2️⃣ 4바이트(32bit) 완성 시 처리
  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4; // b1=LSB, b4=MSB 순서로 dequeue됨
    rxQueue.dequeue(b1);
    // Serial.print(b1); // 디버깅용 Raw 출력 (Source 1)
    rxQueue.dequeue(b2);
    // Serial.print(b2); // 디버깅용 Raw 출력 (Source 1)
    rxQueue.dequeue(b3);
    // Serial.print(b3); // 디버깅용 Raw 출력 (Source 1)
    rxQueue.dequeue(b4);
    // Serial.print(b4); // 디버깅용 Raw 출력 (Source 1)

    // ▼▼▼ 여기서 두 번째 reverseBits 적용 ▼▼▼
    uint8_t reversed_b1 = reverseBits(b1);
    uint8_t reversed_b2 = reverseBits(b2);
    uint8_t reversed_b3 = reverseBits(b3);
    uint8_t reversed_b4 = reverseBits(b4); 

    // ▼▼▼ 패킷 조립 순서 (MSB first: b4, b3, b2, b1 LSB) ▼▼▼
    // (Source 1의 조립 순서를 따름)
    uint32_t packet = ((uint32_t)reversed_b1 << 24) |
                      ((uint32_t)reversed_b2 << 16) |
                      ((uint32_t)reversed_b3 << 8)  |
                      reversed_b4;

    uint8_t rx_crc   = packet & 0xFF;          // 7..0
    uint8_t calc_crc = crc8_24(packet >> 8);
    
    if (rx_crc != 0x00 && rx_crc != calc_crc) {
        crc_err_cnt++;     // 통계용
        blinkThree();      // 디버깅: 깨진 패킷 들어옴
        continue; // 잘못된 패킷이므로 다음 패킷으로
    }
    uint8_t rx_seqbit = (packet >> 8) & 0x1;
    blinkOnce();
    
    // 3️⃣ 디코딩
    uint8_t speed_type = (packet >> 30) & 0x03; // 01 = v
    uint8_t speed_val  = (packet >> 22) & 0xFF;
    uint8_t angle_type = (packet >> 20) & 0x03; // 10 = a
    uint8_t angle_val  = (packet >> 12) & 0xFF;
    uint8_t mode_type  = (packet >> 10) & 0x03; // 11 = s
    uint8_t mode_val   = (packet >> 2)  & 0xFF;

    // 4️⃣ 디코딩된 값으로 로직 수행

    // ----- A. 's' 파라미터(모드) 우선 처리 (Zero) -----
    if (mode_type == 0x03 && mode_val == 0x01) {
        imuUpdate();
        updateHeadingFused();
        yawZeroOffset = lastOutYaw;
        initialHeading = 0.0f;
        targetAngle = 0.0f;
        currentState = IDLE;
        newCommandReceived = false;
        targetSpeed = 0;
        setMotorSpeedDifferential(0, 0);
        moveAtSpeed(0);
        // rxQueue.clear(); // (Queue에 clear 기능이 있다면 호출 권장)
        continue; // 다음 패킷 처리
    }

    // ----- B. 'v'와 'a' 파라미터 처리 -----
    int new_v = 0;
    float new_a = 0.0f;

    if (speed_type == 0x01) { // 01 = v (속도)
        new_v = speed_val;
    }
    // --- ▼▼▼ 수정된 각도 해석 로직 ▼▼▼ ---
    // float new_a = 0.0f; // 이 줄은 위에 이미 선언되었으므로 생략 가능
    if (angle_type == 0x02) { // 10 = a (각도)
        // PC (radar.py)에서 보내는 방식에 맞춰 해석:
        // MSB(최상위 비트)가 1이면 음수, 나머지 7비트는 크기
        if (angle_val & 0x80) {
            // 음수 각도: 최상위 비트를 제외한 나머지 7비트 값에 음수 부호 적용
            new_a = - (float)(angle_val & 0x7F);
        } else {
            // 양수 각도: 값 그대로 사용
            new_a = (float)angle_val;
        }
    }

    // ----- C. 상태 머신 트리거 -----
    if (new_v > 0 || new_a != 0.0f) {
        targetSpeed = constrain(new_v, 0, MAX_MOVE_PWM);
        targetAngle = wrap360(currentHeading + new_a); // 상대 각도 적용
        setMotorSpeedDifferential(0, 0); // (안전을 위해) 일단 정지
        moveAtSpeed(0);

        imuUpdate();
        updateHeadingFused();
        float err = angleDiff(targetAngle, currentHeading);

        if (abs(err) <= ANGLE_DEADBAND) { // 회전 생략
            if (targetSpeed > 0) {
                currentState = MOVING;
                moveStartTime = millis();
                moveAtSpeed(max(targetSpeed, MIN_PWM));
                if (DEBUG) dbgPrintStateLine("[GO]", err);
            } else {
                currentState = IDLE;
            }
        } else { // 회전 시작
            newCommandReceived = true;
            currentState = ROTATING;
        }
    } // end if (new_v > 0 || new_a != 0.0f)
  } // end while (rxQueue.getLength() >= 4)
 
  // 5️⃣ 현재 상태에 따른 동작 수행 (State Machine)
  switch (currentState) {
    case IDLE:
      break;

    case ROTATING:
      if (newCommandReceived) { 
        newCommandReceived = false; 

        // 혹시 이미 목표 각도 근처인지 다시 확인
        imuUpdate();
        updateHeadingFused();
        if (abs(angleDiff(targetAngle, currentHeading)) <= ANGLE_DEADBAND) {
          if (targetSpeed != 0) { 
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM));
          } else { 
            currentState = IDLE;
          }
          break; // ROTATING 상태 종료
        }

        // PID 회전 시작
        if (rotateToAbsAngle(targetAngle)) { // 회전 성공
          if (targetSpeed != 0) { // 직진 속도 있으면 직진 시작
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM));
          } else { // 직진 속도 없으면 완료
            currentState = IDLE;
          }
        } else { // 회전 실패 (타임아웃)
          Serial.println("Rotation Timeout!");
          currentState = IDLE;
        }
      } // end if (newCommandReceived)
      break; // ROTATING 끝

    case MOVING:
      // 2초 직진 타이머
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0); // 정지
        currentState = IDLE; // 대기 상태로
        sendCompletionStatus();
      }
      break; // MOVING 끝
  } // end switch(currentState)

  // 6️⃣ 센서값 및 헤딩 지속적으로 업데이트
  imuUpdate();
  updateHeadingFused();

  // 7️⃣ 디버그 메시지 주기적 출력 (0.2초마다)
  if (DEBUG && millis() - lastDbgMs >= DBG_PERIOD_MS) {
    lastDbgMs = millis();
    float errDbg = angleDiff(targetAngle, currentHeading);
    dbgPrintStateLine("[HDG]", errDbg);
  }

  // 8️⃣ IMU 프레임 수신 카운터 출력 (1초마다)
  if (millis() - lastCountMs >= 1000) {
    lastCountMs = millis();
   // Serial.print("[CNT] acc="); Serial.print(accCnt);
   // Serial.print(" gyr="); Serial.print(gyrCnt);
   // Serial.print(" ang="); Serial.println(angCnt);
    accCnt = gyrCnt = angCnt = 0; // 카운터 리셋
  }

  // 9️⃣ 메인 루프 지연 (약 50Hz)
  delay(20);
}
