#include <AltSoftSerial.h>    // WT901용 (UNO: RX=8, TX=9 고정)
AltSoftSerial imuSerial;      // IMU on D8(RX), D9(TX)
#define IMU_SERIAL imuSerial

// ===== 전역 구조체 (WitMotion 형식 유지) =====
struct SAngle { float Roll, Pitch, Yaw; };
struct SAcc   { float X, Y, Z; };
struct SGyro  { float X, Y, Z; };
SAngle stcAngle; // 최종 각도 (deg)
SAcc   stcAcc;   // 최종 가속도 (g)
SGyro  stcGyro;  // 최종 각속도 (deg/s)

// ===== 보조 벡터 (기존 코드 호환용) =====
float Axyz[3] = {0}, Gxyz[3] = {0};

// ===== 헤딩/상태 =====
float currentHeading = 0.0f;    // 최종 출력 헤딩(도)
float initialHeading = 0.0f;    // 시작시 기준 헤딩(도)
float targetAngle    = 0.0f;    // 목표 절대 헤딩(도)
int   targetSpeed    = 0;
bool  newCommandReceived = false;

// Zero(영점) 기능용
float yawZeroOffset = 0.0f;   // 현재 헤딩을 0으로 만들기 위한 오프셋
float lastOutYaw    = 0.0f;   // LPF 전개 후 내부 outYaw(영점 계산에 사용)

// 상태머신
enum RobotState { IDLE, ROTATING, MOVING };
RobotState currentState = IDLE;
unsigned long moveStartTime = 0;
// ===== PID =====
float Kp = 4.0f;
float Ki = 0.0f;
float Kd = 0.1f;

// ===== 모터 핀/파라미터 =====
#define ENA 0x08
#define IN1 0x10
#define IN2 0x20
#define ENB 0x08
#define IN3 0x10
#define IN4 0x20


// 회전 속도는 pid 제어에 따라 min ~ max, 직진 속도는 입력값
const int MIN_PWM = 70;
const int MAX_PWM = 100; // 회전 최대 속도 (setMotorSpeedDifferential에서 사용)
const int MAX_MOVE_PWM = 100; // 직진 최대 속도 (moveAtSpeed에서 사용)
const int INITIAL_MOVE_SPEED = 50;
const int INITIAL_MOVE_DURATION = 500; // ms
const unsigned long MOVE_DURATION = 2000; // ms

// ===== 제어 유틸 =====
static inline float wrap360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}
static inline float angleDiff(float target, float current) {
  float diff = target - current;
  // -180 ~ +180 범위로 정규화 (가장 짧은 경로)
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

// 회전 데드밴드(이 정도면 회전 생략)
const float ANGLE_DEADBAND = 3.0f;

// 움직임 플래그 (모터 명령이 실제로 나가는지 판단)
bool moving = false;

// 출력 LPF 계수 (헤딩 안정화)
const float LPF_ALPHA = 0.25f;

// ===== 모터 제어 =====
void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  int pwmValue = abs(constrain(speed, -255, 255)); // 0~255 사이의 절대값

  if(EN_pin == ENA) { // ENA는 D3 (OC2B) 핀
    if (speed > 0) { // 전진
      digitalWrite(IN1_pin, 1);
      digitalWrite(IN2_pin, 0);
    } else if (speed < 0) { // 후진
      digitalWrite(IN1_pin, 0);
      digitalWrite(IN2_pin, 1);
    } else { // 정지
      digitalWrite(IN1_pin, 0);
      digitalWrite(IN2_pin, 0); // (0,0) 또는 (1,1)로 정지 (Low/Low 방식)
    }
    OCR2B = pwmValue; // D3 핀의 PWM 제어

  } else if(EN_pin == ENB) { // ENB는 D11 (OC2A) 핀
    if (speed > 0) { // 전진
      digitalWrite(IN1_pin, 1);
      digitalWrite(IN2_pin, 0);
    } else if (speed < 0) { // 후진
      digitalWrite(IN1_pin, 0);
      digitalWrite(IN2_pin, 1);
    } else { // 정지
      digitalWrite(IN1_pin, 0);
      digitalWrite(IN2_pin, 0);
    }
    OCR2A = pwmValue; // D11 핀의 PWM 제어
  }
}

// 개선된 탱크 턴 함수 - 회전 방향 문제 해결
void setMotorSpeedDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  moving = (abs(leftSpeed) >= MIN_PWM || abs(rightSpeed) >= MIN_PWM);
  if (!moving) {
    driveOneMotor(ENA, IN1, IN2, 0);
    driveOneMotor(ENB, IN3, IN4, 0);
    return;
  }
  // MIN_PWM 데드존 처리
  if (abs(leftSpeed) < MIN_PWM && leftSpeed != 0) {
    leftSpeed = (leftSpeed > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(rightSpeed) < MIN_PWM && rightSpeed != 0) {
    rightSpeed = (rightSpeed > 0) ? MIN_PWM : -MIN_PWM;
  }
  // 직접 모터 제어 (개선된 방향)
  driveOneMotor(ENA, IN1, IN2, leftSpeed);   // 왼쪽 모터
  driveOneMotor(ENB, IN3, IN4, rightSpeed);  // 오른쪽 모터
}
void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  // ===> 주의: 방향 반전 로직입니다. v50 입력 시 후진합니다. <===
  int corrected = -speed; // 의도한 동작이 아니라면 수정 필요 (corrected = speed;)
  moving = (abs(corrected) >= MIN_PWM);
  driveOneMotor(ENA, IN1, IN2, corrected);
  driveOneMotor(ENB, IN3, IN4, corrected);
}


// 여긴 수정 X
// =====================================================
// ================= WT901 파싱 파트 ====================
// =====================================================
// WT901 Baud (모듈 설정과 반드시 동일)
const unsigned long WT901_BAUD = 9600;
// WT901 프레임: 11바이트, 0x55 0x51(Acc)/0x52(Gyro)/0x53(Angle)
uint8_t imuFrame[11];
uint8_t imuIdx = 0;
inline bool wt901CheckSumOK(const uint8_t* f) {
  uint8_t sum = 0;
  for (int i = 0; i < 10; ++i) sum += f[i];
  return (sum == f[10]);
}
void imuSetup() {
  IMU_SERIAL.begin(WT901_BAUD);  // AltSoftSerial (핀 8, 9 고정)
}
// WT901 바이트 스트림을 파싱해 stcAcc/stcGyro/stcAngle 갱신
void imuPollWT901() {
  while (IMU_SERIAL.available()) {
    uint8_t c = (uint8_t)IMU_SERIAL.read();
    // 시작 바이트(0x55) 찾기
    if (imuIdx == 0) {
      if (c != 0x55) continue;
      imuFrame[imuIdx++] = c;
      continue;
    }
    // 프레임 버퍼 채우기
    imuFrame[imuIdx++] = c;
    // 두 번째 바이트(패킷 타입) 확인
    if (imuIdx == 2) {
      if (imuFrame[1] != 0x51 && imuFrame[1] != 0x52 && imuFrame[1] != 0x53) {
        // 잘못된 패킷 타입이면 재동기화 시도
        imuIdx = (c == 0x55) ? 1 : 0; // 현재 바이트가 0x55면 인덱스 1부터 다시 시작
      }
    }
    // 프레임 11바이트 다 받으면 처리
    if (imuIdx == 11) {
      imuIdx = 0; // 인덱스 초기화
      if (!wt901CheckSumOK(imuFrame)) continue; // 체크섬 검사
      // 데이터 해석 함수 (람다식, 리틀 엔디언)
      auto s16 = [&](int lo, int hi) -> int16_t {
        return (int16_t)(((uint16_t)imuFrame[hi] << 8) | (uint16_t)imuFrame[lo]);
      };
      // 패킷 타입에 따라 데이터 저장
      if (imuFrame[1] == 0x51) { // 가속도
        stcAcc.X = (float)s16(2, 3) / 32768.0f * 16.0f; // [g]
        stcAcc.Y = (float)s16(4, 5) / 32768.0f * 16.0f;
        stcAcc.Z = (float)s16(6, 7) / 32768.0f * 16.0f;
        Axyz[0] = stcAcc.X; Axyz[1] = stcAcc.Y; Axyz[2] = stcAcc.Z;
        accCnt++;
      } else if (imuFrame[1] == 0x52) { // 각속도
        stcGyro.X = (float)s16(2, 3) / 32768.0f * 2000.0f; // [deg/s]
        stcGyro.Y = (float)s16(4, 5) / 32768.0f * 2000.0f;
        stcGyro.Z = (float)s16(6, 7) / 32768.0f * 2000.0f;
        Gxyz[0] = stcGyro.X; Gxyz[1] = stcGyro.Y; Gxyz[2] = stcGyro.Z;
        gyrCnt++;
      } else if (imuFrame[1] == 0x53) { // 각도
        stcAngle.Roll  = (float)s16(2, 3) / 32768.0f * 180.0f; // [deg]
        stcAngle.Pitch = (float)s16(4, 5) / 32768.0f * 180.0f;
        stcAngle.Yaw   = (float)s16(6, 7) / 32768.0f * 180.0f; // [-180 ~ +180]
        angCnt++;
      }
    } // end if (imuIdx == 11)
  } // end while (IMU_SERIAL.available())
}
// 기존 함수명 유지: 센서 갱신 (WT901 데이터 폴링)
void imuUpdate() {
  imuPollWT901();
}
// WT901의 Yaw를 사용해 헤딩 산출(LPF)
// + Zero 기능 반영
// + 방향 반전 (시계방향(+)으로 각도 증가)
void updateHeadingFused() {
  static bool init = true;
  static float outYaw = 0.0f; // LPF 적용된 내부 Yaw (0~360)
  // WT901 Yaw [-180~+180] -> [0~360] 변환
  float rawYaw360 = wrap360(stcAngle.Yaw < 0 ? (stcAngle.Yaw + 360.0f) : stcAngle.Yaw);
  // 방향 반전 (시계반대방향+ 되도록) - IMU 방향 문제 해결
  float measured = rawYaw360; // IMU 방향을 그대로 사용
  // LPF (Low Pass Filter) 적용
  if (init) {
    outYaw = measured; // 첫 값은 그대로 사용
    init = false;
  } else {
    float d = angleDiff(measured, outYaw); // 측정값과 이전 필터값의 차이
    outYaw = wrap360(outYaw + LPF_ALPHA * d); // 차이의 일부(ALPHA)만 반영
  }
  lastOutYaw = outYaw; // Zero(영점) 설정 시 사용
  // Zero(영점) 오프셋 적용
  currentHeading = wrap360(outYaw - yawZeroOffset); // 최종 헤딩 계산
}

// ===== PID 회전 =====
bool rotateToAbsAngle(float targetAbsAngle) {
  float error = 0.0f;
  float integral = 0.0f, lastError = 0.0f, derivative = 0.0f;
  int correction = 0;

  // const float tolerance = 2.0f; // ANGLE_DEADBAND 사용으로 불필요
  unsigned long lastTime = micros(), startTime = millis();
  unsigned long lastPrintMs = millis();
  while (true) {
    // 타임아웃 (15초)
    if (millis() - startTime > 15000UL) {
      setMotorSpeedDifferential(0, 0); // 정지
      return false; // 실패
    }
    // 시간 계산 (초 단위)
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01f; // 0 방지

    // 센서 읽기 및 헤딩 계산
    imuUpdate();          // WT901 데이터 읽기
    updateHeadingFused();  // currentHeading 갱신

    // 오차 계산
    error = angleDiff(targetAbsAngle, currentHeading);

    // 목표 도달 확인 (Deadband 이내면 성공)
    if (abs(error) <= ANGLE_DEADBAND) {
      setMotorSpeedDifferential(0, 0); // 정지
      return true; // 성공
    }
    // PID 제어 계산
    integral += error * deltaTime;
    // 적분 항 제한 (필요시 추가)
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = (int)round(Kp * error + Ki * integral + Kd * derivative);
    // PID 출력값 -> 모터 속도 변환
    // MIN_PWM 데드존 처리
    if (abs(correction) < MIN_PWM) {
      correction = (correction >= 0) ? MIN_PWM : -MIN_PWM;
    }
    // MAX_PWM 제한
    correction = constrain(correction, -MAX_PWM, MAX_PWM);
    // 모터 구동 (탱크 턴) - IMU 방향에 맞춘 회전 방향
    // error > 0: 시계방향 회전 필요, error < 0: 시계반대방향 회전 필요
    int leftSpeed, rightSpeed;
    if (error > 0) {
      // 시계방향 회전: 왼쪽 전진, 오른쪽 후진
      leftSpeed = abs(correction);
      rightSpeed = -abs(correction);
    } else {
      // 시계반대방향 회전: 왼쪽 후진, 오른쪽 전진  
      leftSpeed = -abs(correction);
      rightSpeed = abs(correction);
    }

    setMotorSpeedDifferential(leftSpeed, rightSpeed);
    delay(10); // 제어 루프 주기 (약 100Hz)
  } // end while
}

// 지정된 시리얼 포트(Stream)로부터 명령 한 줄 읽고 처리
void handleInputFrom(Stream &port) {
  while (port.available()) {
    char incomingByte = port.read();
    if (&port == &mySerial) {
      Serial.write(incomingByte);
    }
    // ----- Zero 단축키: 'z' 또는 'Z' 처리 -----
    if (incomingByte == 'z' || incomingByte == 'Z') {
      imuUpdate(); 
      updateHeadingFused(); // 센서값 최신화
      // 완전 리셋
      yawZeroOffset = lastOutYaw;      // 현재 내부 Yaw 값을 오프셋으로 저장
      initialHeading = 0.0f;           // 초기 헤딩을 0으로 설정
      targetAngle = 0.0f;            // 목표 각도도 0으로 리셋
      // 상태도 초기화
      currentState = IDLE;
      newCommandReceived = false;
      targetSpeed = 0;
      // 모터 정지
      setMotorSpeedDifferential(0, 0);
      moveAtSpeed(0);
      uartBufferIndex = 0; // 명령어 버퍼 초기화
      continue; // 다음 바이트 처리로
    }
    // ----- 줄바꿈 문자(\n 또는 \r) 처리 -----
    if (incomingByte == '\n' || incomingByte == '\r') {
      if (uartBufferIndex > 0) { // 버퍼에 내용이 있으면
        uartBuffer[uartBufferIndex] = '\0'; // 문자열 종료 처리
        int receivedSpeed; float receivedAngle; int receivedS;
        // "v...a...s..." 형식 파싱 시도
        if (parseCommand(uartBuffer, receivedSpeed, receivedAngle, receivedS)) {
          // 파싱 성공
          targetSpeed = constrain(receivedSpeed, 0, MAX_MOVE_PWM); // 직진 속도 설정 (0~150)
          // 상대 각도 처리: 현재 헤딩에서 상대 각도만큼 회전
          // +a는 왼쪽(시계반대), -a는 오른쪽(시계방향)
          // IMU 방향에 맞춰 각도 계산
          targetAngle = wrap360(currentHeading + receivedAngle);

          // 일단 정지
          setMotorSpeedDifferential(0, 0);
          moveAtSpeed(0);
          // 현재 각도 다시 확인
          imuUpdate();
          updateHeadingFused();
          float err = angleDiff(targetAngle, currentHeading); // 목표와의 오차 계산
          // 오차가 Deadband 이내면 회전 생략
          if (abs(err) <= ANGLE_DEADBAND) {
            if (targetSpeed > 0) { // v값이 있으면 바로 직진 시작
              currentState = MOVING;
              moveStartTime = millis();
              int startSpeed = max(targetSpeed, MIN_PWM); // 최소 속도 보장
              moveAtSpeed(startSpeed);
              if (DEBUG) dbgPrintStateLine("[GO]", err);
            } else { // v값이 0이면 그냥 완료 처리
              sendHeadingFrame(); // 완료 신호 전송
              currentState = IDLE;
            }
          } else { // 오차가 크면 회전 시작
            newCommandReceived = true; // 새 명령 플래그 설정
            currentState = ROTATING; // 회전 상태로 변경
          }
        } // end if (parseCommand)
      } // end if (uartBufferIndex > 0)
      uartBufferIndex = 0; // 버퍼 비우기
    }
    // ----- 버퍼 채우기 -----
    else if (uartBufferIndex < (UART_BUFFER_SIZE - 1)) { // 버퍼 공간 남았으면
      uartBuffer[uartBufferIndex++] = incomingByte; // 버퍼에 추가
    }
    // 버퍼 꽉 찼으면 무시 (오버플로우 방지)
  } // end while (port.available())
}



// ===== Arduino 기본 Setup =====
void setup() {
  imuSetup();                       // WT901 센서용 (9600)

  // 모터 핀 출력 설정
  DDRD |= (ENA | IN1 | IN2);
  DDRB |= (ENB | IN3 | IN4);

  // pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  // pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  // pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  TCCR2A = 0; // TCCR2A 레지스터 초기화
  TCCR2B = 0; // TCCR2B 레지스터 초기화
  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1)
  TCCR2B |= (0 << CS22) | (1 << CS21) | (1 << CS20);
  OCR2A = 0;
  OCR2B = 0;
  

  // 시작 시 잠깐 움직여서 모터 풀어주기
  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);
  delay(500);

  // ===== 초기 헤딩 설정 (안정화 대기) =====
  unsigned long t0 = millis();
  uint16_t angSamples = 0;
  float lastYawSample = 9999.0f;
  while (millis() - t0 < 2000) {  // 최대 2초 대기
    imuUpdate(); // 센서 읽기
    updateHeadingFused(); // 헤딩 계산
    // Yaw 값이 안정화 (최소 5번 다른 값 수신)될 때까지 기다림
    if (stcAngle.Yaw != lastYawSample) {
      lastYawSample = stcAngle.Yaw;
      angSamples++;
      if (angSamples >= 5) break; // 안정화 완료
    }
    delay(10);
  }
  // 현재 방향을 0도로 설정 (선택 사항)
  yawZeroOffset = lastOutYaw;     // 현재 내부 Yaw 값을 오프셋으로 사용
  updateHeadingFused();           // currentHeading을 0 근처로 갱신
  initialHeading = currentHeading; // 현재 각도(0 근처)를 초기 기준으로 설정
  targetAngle    = initialHeading;
}
// ===== Arduino 기본 Loop =====
void loop() {
  switch (currentState) {
    case IDLE:
      break;
    case ROTATING:
      if (newCommandReceived) { // 새 회전 명령 들어왔으면
        newCommandReceived = false; // 플래그 해제
        // 혹시 이미 목표 각도 근처인지 다시 확인 (회전 생략)
        imuUpdate(); updateHeadingFused();
        if (abs(angleDiff(targetAngle, currentHeading)) <= ANGLE_DEADBAND) {
          if (targetSpeed != 0) { // 직진 속도 있으면 바로 직진
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM));
          } else { // 직진 속도 없으면 완료
            sendHeadingFrame();
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
            sendHeadingFrame();
            currentState = IDLE;
          }
        } else { // 회전 실패 (타임아웃)
          sendHeadingFrame(); // 현재 각도 보고
          currentState = IDLE;
        }
      } // end if (newCommandReceived)
      break; // ROTATING 끝
    case MOVING:
      // 2초 직진 타이머
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0); // 정지
        sendHeadingFrame(); // 완료 보고
        currentState = IDLE; // 대기 상태로
      }
      break; // MOVING 끝
  } // end switch(currentState)
  // 3. 센서값 및 헤딩 지속적으로 업데이트
  imuUpdate();
  updateHeadingFused();
  
  // 6. 메인 루프 지연 (약 50Hz)
  delay(20);
}
