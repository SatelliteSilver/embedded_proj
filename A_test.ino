#include <AltSoftSerial.h>     // WT901용 (UNO: RX=8, TX=9 고정)
#include <SoftwareSerial.h>    // 명령 포트용 (D6, D7)

AltSoftSerial imuSerial;       // IMU on D8(RX), D9(TX)
#define IMU_SERIAL imuSerial

// Arduino B와 통신(명령/피드백)
SoftwareSerial mySerial(6, 7); // RX, TX

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
float currentHeading = 0.0f;   // 최종 출력 헤딩(도)
float initialHeading = 0.0f;   // 시작시 기준 헤딩(도)
float targetAngle    = 0.0f;   // 목표 절대 헤딩(도)
int   targetSpeed    = 0;
bool  newCommandReceived = false;

// Zero(영점) 기능용
float yawZeroOffset = 0.0f;    // 현재 헤딩을 0으로 만들기 위한 오프셋
float lastOutYaw    = 0.0f;    // LPF 전개 후 내부 outYaw(영점 계산에 사용)

// 상태머신
enum RobotState { IDLE, ROTATING, MOVING };
RobotState currentState = IDLE;

unsigned long moveStartTime = 0;

// ===== PID =====
float Kp = 4.0f;
float Ki = 0.0f;
float Kd = 0.1f;

// ===== 모터 핀/파라미터 =====
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 11
#define IN3 12
#define IN4 13

const int MIN_PWM = 40;
const int MAX_PWM = 250;
const int MAX_MOVE_PWM = 150;
const int INITIAL_MOVE_SPEED = 50;
const int INITIAL_MOVE_DURATION = 500; // ms
const unsigned long MOVE_DURATION = 2000; // ms

// ===== UART & 디버그 =====
#define UART_BAUD_RATE 115200
#define UART_BUFFER_SIZE 32
char uartBuffer[UART_BUFFER_SIZE];
byte uartBufferIndex = 0;

#define DEBUG 1
unsigned long lastDbgMs = 0;
const unsigned long DBG_PERIOD_MS = 200;

// 프레임 카운터(1초마다 표시)
uint32_t accCnt=0, gyrCnt=0, angCnt=0, lastCountMs=0;

// ===== 제어 유틸 =====
static inline float wrap360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}
static inline float angleDiff(float target, float current) {
  float diff = wrap360(target) - wrap360(current);
  if (diff >= 180.0f)  diff -= 360.0f;
  if (diff < -180.0f)  diff += 360.0f;
  return diff;
}

// 회전 데드밴드(이 정도면 회전 생략)
const float ANGLE_DEADBAND = 3.0f;

// 움직임 플래그 (모터 명령이 실제로 나가는지 판단)
bool moving = false;

// 출력 LPF 계수 (헤딩 안정화)
const float LPF_ALPHA = 0.25f;

// ===== 디버그 1줄 출력 =====
void dbgPrintStateLine(const char* tag, float err) {
  if (!DEBUG) return;
  Serial.print(tag); Serial.print(" | STATE=");
  Serial.print((int)currentState);        // 0:IDLE, 1:ROTATING, 2:MOVING
  Serial.print(" v=");  Serial.print(targetSpeed);
  Serial.print(" hdg="); Serial.print(currentHeading, 1);
  Serial.print(" tgt="); Serial.print(targetAngle, 1);
  Serial.print(" err="); Serial.println(err, 1);
}

// ===== 모터 제어 =====
void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, speed);
  } else if (speed < 0) {
    digitalWrite(IN1_pin, LOW); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, -speed);
  } else {
    digitalWrite(IN1_pin, LOW); digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, 0);
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

  int actualLeftSpeed = 0, actualRightSpeed = 0;
  int turnPWM;

  if (abs(rightSpeed) >= abs(leftSpeed)) {   // 우회전 쪽 우선
    turnPWM = max(MIN_PWM, abs(rightSpeed));
    actualLeftSpeed = -turnPWM; actualRightSpeed =  turnPWM;
  } else {                                   // 좌회전
    turnPWM = max(MIN_PWM, abs(leftSpeed));
    actualLeftSpeed =  turnPWM; actualRightSpeed = -turnPWM;
  }

  driveOneMotor(ENA, IN1, IN2, actualLeftSpeed);
  driveOneMotor(ENB, IN3, IN4, actualRightSpeed);
}

void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  int corrected = -speed; // 전진/후진 방향 반전 필요 시 사용
  moving = (abs(corrected) >= MIN_PWM);
  driveOneMotor(ENA, IN1, IN2, corrected);
  driveOneMotor(ENB, IN3, IN4, corrected);
}

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
  IMU_SERIAL.begin(WT901_BAUD);  // AltSoftSerial
}

// WT901 바이트 스트림을 파싱해 stcAcc/stcGyro/stcAngle 갱신
void imuPollWT901() {
  while (IMU_SERIAL.available()) {
    uint8_t c = (uint8_t)IMU_SERIAL.read();

    if (imuIdx == 0) {
      if (c != 0x55) continue;
      imuFrame[imuIdx++] = c;
      continue;
    }

    imuFrame[imuIdx++] = c;

    // 두 번째 바이트가 0x51/0x52/0x53가 아니면 프레임 재동기화
    if (imuIdx == 2) {
      if (imuFrame[1] != 0x51 && imuFrame[1] != 0x52 && imuFrame[1] != 0x53) {
        imuIdx = (c == 0x55) ? 1 : 0;
      }
    }

    if (imuIdx == 11) {
      imuIdx = 0;
      if (!wt901CheckSumOK(imuFrame)) continue;

      // 데이터 해석 (리틀엔디언)
      auto s16 = [&](int lo, int hi) -> int16_t {
        return (int16_t)((imuFrame[hi] << 8) | imuFrame[lo]);
      };

      if (imuFrame[1] == 0x51) {
        // Acc: raw/32768 * 16  [g]
        stcAcc.X = (float)s16(2,3) / 32768.0f * 16.0f;
        stcAcc.Y = (float)s16(4,5) / 32768.0f * 16.0f;
        stcAcc.Z = (float)s16(6,7) / 32768.0f * 16.0f;
        Axyz[0] = stcAcc.X; Axyz[1] = stcAcc.Y; Axyz[2] = stcAcc.Z;
        accCnt++;
      } else if (imuFrame[1] == 0x52) {
        // Gyro: raw/32768 * 2000 [deg/s]
        stcGyro.X = (float)s16(2,3) / 32768.0f * 2000.0f;
        stcGyro.Y = (float)s16(4,5) / 32768.0f * 2000.0f;
        stcGyro.Z = (float)s16(6,7) / 32768.0f * 2000.0f;
        Gxyz[0] = stcGyro.X; Gxyz[1] = stcGyro.Y; Gxyz[2] = stcGyro.Z;
        gyrCnt++;
      } else if (imuFrame[1] == 0x53) {
        // Angle: raw/32768 * 180 [deg]  (Yaw 범위 -180~+180)
        stcAngle.Roll  = (float)s16(2,3) / 32768.0f * 180.0f;
        stcAngle.Pitch = (float)s16(4,5) / 32768.0f * 180.0f;
        stcAngle.Yaw   = (float)s16(6,7) / 32768.0f * 180.0f;
        angCnt++;
      }
    }
  }
}

// 기존 함수명 유지: 센서 갱신(Acc/Gyro/Angle 모두 반영)
void imuUpdate() {
  imuPollWT901();   // AltSoftSerial은 listen() 필요 없음
}

// WT901의 Yaw를 사용해 헤딩 산출(LPF)
//  + B) Zero 기능 반영 (yawZeroOffset 적용, lastOutYaw 갱신)
void updateHeadingFused() {
  static bool init = true;
  static float outYaw = 0.0f;

  // WT901 Yaw는 -180~+180 → 0~360으로 정규화
  float measured = wrap360(stcAngle.Yaw < 0 ? (stcAngle.Yaw + 360.0f) : stcAngle.Yaw);

  if (init) {
    outYaw = measured;
    init = false;
  } else {
    float d = angleDiff(measured, outYaw);
    outYaw = wrap360(outYaw + LPF_ALPHA * d);
  }

  lastOutYaw = outYaw; // 영점 설정 시 사용

  // Zero 오프셋 적용
  currentHeading = wrap360(outYaw - yawZeroOffset);
}

// 각도만 전송 (절대 헤딩) — 회전/주행 완료 시점에만 호출
void sendHeadingFrame() {
  mySerial.print("h,");
  mySerial.print(currentHeading, 1);
  mySerial.print('\n');
}

// ===== PID 회전 =====
bool rotateToAbsAngle(float targetAbsAngle) {
  float error = 0.0f;
  float integral = 0.0f, lastError = 0.0f, derivative = 0.0f;
  int correction = 0;

  const float tolerance = 2.0f;
  unsigned long lastTime = micros(), startTime = millis();
  unsigned long lastPrintMs = millis();

  while (true) {
    if (millis() - startTime > 15000UL) {    // 타임아웃
      setMotorSpeedDifferential(0, 0);
      return false;
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0f;
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01f;

    imuUpdate();           // WT901 프레임 처리
    updateHeadingFused();  // 헤딩 갱신

    error = angleDiff(targetAbsAngle, currentHeading);

    if (DEBUG && millis() - lastPrintMs >= 100) {
      lastPrintMs = millis();
      dbgPrintStateLine("[ROT]", error);
    }

    if (abs(error) <= ANGLE_DEADBAND) {
      setMotorSpeedDifferential(0, 0);
      return true;
    }

    // PID
    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;

    correction = (int)round(Kp * error + Ki * integral + Kd * derivative);

    if (abs(correction) < MIN_PWM) {
      correction = (correction >= 0) ? MIN_PWM : -MIN_PWM;
    }
    correction = constrain(correction, -MAX_PWM, MAX_PWM);

    int leftSpeed  = -correction;
    int rightSpeed =  correction;
    setMotorSpeedDifferential(leftSpeed, rightSpeed);

    delay(10);
  }
}

// ===== 명령 파싱/입력 =====
bool parseCommand(const char* command, int &speed, float &angle, int &s_param) {
  const char* v_ptr = strchr(command, 'v');
  const char* a_ptr = strchr(command, 'a');
  const char* s_ptr = strchr(command, 's');
  if (!v_ptr || !a_ptr || !s_ptr) return false;

  speed   = atoi(v_ptr + 1);
  angle   = atof(a_ptr + 1);
  s_param = atoi(s_ptr + 1);
  return true;
}

void handleInputFrom(Stream &port) {
  while (port.available()) {
    char incomingByte = port.read();
    if (&port == &mySerial) Serial.write(incomingByte); // echo to USB

    // ----- B) Zero 단축키: 'z' 또는 'Z' -----
    if (incomingByte == 'z' || incomingByte == 'Z') {
      // 현재 outYaw를 영점 기준으로 사용
      imuUpdate(); updateHeadingFused(); // 최신화
      yawZeroOffset = lastOutYaw;
      initialHeading = currentHeading;   // 이제 0 근처
      targetAngle    = initialHeading;
      Serial.println("[ZERO] heading set to 0 deg");
      uartBufferIndex = 0; // 버퍼 초기화
      continue;
    }

    if (incomingByte == '\n' || incomingByte == '\r') {
      if (uartBufferIndex > 0) {
        uartBuffer[uartBufferIndex] = '\0';
        int receivedSpeed; float receivedAngle; int receivedS;
        if (parseCommand(uartBuffer, receivedSpeed, receivedAngle, receivedS)) {
          targetSpeed = constrain(receivedSpeed, 0, MAX_MOVE_PWM);
          targetAngle = wrap360(initialHeading + receivedAngle);

          if (DEBUG) {
            Serial.print("[CMD] v="); Serial.print(targetSpeed);
            Serial.print(" a_rel=");  Serial.print(receivedAngle, 1);
            Serial.print(" -> tgtAbs="); Serial.println(targetAngle, 1);
          }

          setMotorSpeedDifferential(0, 0);
          moveAtSpeed(0);

          imuUpdate();
          updateHeadingFused();
          float err = angleDiff(targetAngle, currentHeading);

          if (abs(err) <= ANGLE_DEADBAND) {
            if (targetSpeed > 0) {
              currentState = MOVING;
              moveStartTime = millis();
              int startSpeed = max(targetSpeed, MIN_PWM); // 최소 PWM 보장
              moveAtSpeed(startSpeed);
              if (DEBUG) dbgPrintStateLine("[GO]", err);
            } else {
              sendHeadingFrame();
              currentState = IDLE;
            }
          } else {
            newCommandReceived = true;
            currentState = ROTATING;
          }
        }
      }
      uartBufferIndex = 0;
    } else if (uartBufferIndex < (UART_BUFFER_SIZE - 1)) { // ★ 버그 수정: BAUD_RATE -> BUFFER_SIZE
      uartBuffer[uartBufferIndex++] = incomingByte;
    }
  }
}

void handleUartInput() {
  // 명령 포트(6,7) 수신
  mySerial.listen();
  handleInputFrom(mySerial);
  handleInputFrom(Serial);   // USB 시리얼도 명령 허용(옵션)
}

// ===== Arduino 기본 =====
void setup() {
  Serial.begin(UART_BAUD_RATE);     // PC 모니터 115200
  mySerial.begin(115200);           // 명령 링크 115200
  imuSetup();                       // WT901 9600

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // 기계 유격/정지 마찰 풀기
  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);
  delay(500);

  // ===== A) 초기 헤딩을 "유효 각도 수신 후"에 설정 =====
  unsigned long t0 = millis();
  uint16_t angSamples = 0;
  float lastYawSample = 9999.0f;
  while (millis() - t0 < 2000) {  // 최대 2초 대기
    imuUpdate();
    updateHeadingFused();
    if (stcAngle.Yaw != lastYawSample) {
      lastYawSample = stcAngle.Yaw;
      angSamples++;
      if (angSamples >= 5) break; // 최소 5회 각도 프레임
    }
    delay(10);
  }

  // 현재 헤딩을 0으로 쓰고 싶으면 아래 두 줄로 영점도 같이 맞춘다(선택)
  yawZeroOffset = lastOutYaw;            // 현재 outYaw를 기준 0으로
  updateHeadingFused();                  // currentHeading 갱신(거의 0 근처)

  initialHeading = currentHeading;       // 0 근처
  targetAngle    = initialHeading;

  Serial.println("Setup complete. Ready for commands.");
}

void loop() {
  handleUartInput();

  switch (currentState) {
    case IDLE:
      break;

    case ROTATING:
      if (newCommandReceived) {
        newCommandReceived = false;

        imuUpdate(); updateHeadingFused();
        if (abs(angleDiff(targetAngle, currentHeading)) <= ANGLE_DEADBAND) {
          if (targetSpeed != 0) {
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM)); // 최소 PWM 보장
          } else {
            sendHeadingFrame();
            currentState = IDLE;
          }
          break;
        }

        if (rotateToAbsAngle(targetAngle)) {
          if (targetSpeed != 0) {
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM)); // 최소 PWM 보장
          } else {
            sendHeadingFrame();
            currentState = IDLE;
          }
        } else {
          sendHeadingFrame();
          currentState = IDLE;
        }
      }
      break;

    case MOVING:
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0);
        sendHeadingFrame();
        currentState = IDLE;
      }
      break;
  }

  imuUpdate();
  updateHeadingFused();

  if (DEBUG && millis() - lastDbgMs >= DBG_PERIOD_MS) {
    lastDbgMs = millis();
    float errDbg = angleDiff(targetAngle, currentHeading);
    dbgPrintStateLine("[HDG]", errDbg);
  }

  // 1초마다 프레임 카운터 출력
  if (millis() - lastCountMs >= 1000) {
    lastCountMs = millis();
    Serial.print("[CNT] acc="); Serial.print(accCnt);
    Serial.print(" gyr="); Serial.print(gyrCnt);
    Serial.print(" ang="); Serial.println(angCnt);
    accCnt=gyrCnt=angCnt=0;
  }

  delay(20);
}
