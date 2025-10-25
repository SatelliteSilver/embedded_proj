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



// 회전 속도는 pid 제어에 따라 min ~ max, 직진 속도는 입력값

const int MIN_PWM = 70;

const int MAX_PWM = 80; // 회전 최대 속도 (setMotorSpeedDifferential에서 사용)

const int MAX_MOVE_PWM = 150; // 직진 최대 속도 (moveAtSpeed에서 사용)

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

uint32_t accCnt = 0, gyrCnt = 0, angCnt = 0, lastCountMs = 0;



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

  if (speed > 0) { // 전진

    digitalWrite(IN1_pin, HIGH);

    digitalWrite(IN2_pin, LOW);

    analogWrite(EN_pin, speed);

  } else if (speed < 0) { // 후진

    digitalWrite(IN1_pin, LOW);

    digitalWrite(IN2_pin, HIGH);

    analogWrite(EN_pin, -speed);

  } else { // 정지 (Low/Low 방식)

    digitalWrite(IN1_pin, LOW);

    digitalWrite(IN2_pin, LOW);

    analogWrite(EN_pin, 0);

  }

}



// ===> 주의: 이 함수는 이전 버전 함수입니다. 방향 문제가 해결된 탱크 턴 함수 사용을 권장합니다. <===

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



  // 이 로직은 PID 결과(correction)를 어떻게 해석하느냐에 따라 방향이 달라질 수 있습니다.

  if (abs(rightSpeed) >= abs(leftSpeed)) {   // 우회전 시도 시 rightSpeed 크기가 더 크다고 가정

    turnPWM = max(MIN_PWM, abs(rightSpeed));

    actualLeftSpeed = -turnPWM; // 왼쪽 뒤로

    actualRightSpeed =  turnPWM; // 오른쪽 앞으로

  } else {                                   // 좌회전 시도 시 leftSpeed 크기가 더 크다고 가정

    turnPWM = max(MIN_PWM, abs(leftSpeed));

    actualLeftSpeed =  turnPWM; // 왼쪽 앞으로

    actualRightSpeed = -turnPWM; // 오른쪽 뒤로

  }



  driveOneMotor(ENA, IN1, IN2, actualLeftSpeed);

  driveOneMotor(ENB, IN3, IN4, actualRightSpeed);

}



void moveAtSpeed(int speed) {

  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);



  // ===> 주의: 방향 반전 로직입니다. v50 입력 시 후진합니다. <===

  int corrected = -speed; // 의도한 동작이 아니라면 수정 필요 (corrected = speed;)



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



  // 방향 반전 (시계방향+ 되도록)

  float measured = wrap360(360.0f - rawYaw360);



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



// Arduino B로 현재 헤딩 전송

void sendHeadingFrame() {

  mySerial.print("h,"); // 헤더

  mySerial.print(currentHeading, 1); // 현재 헤딩 (소수점 1자리)

  mySerial.print('\n'); // 종료 문자

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

    imuUpdate();           // WT901 데이터 읽기

    updateHeadingFused();  // currentHeading 갱신



    // 오차 계산

    error = angleDiff(targetAbsAngle, currentHeading);



    // 디버그 출력 (0.1초마다)

    if (DEBUG && millis() - lastPrintMs >= 100) {

      lastPrintMs = millis();

      dbgPrintStateLine("[ROT]", error);

    }



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



    // 모터 구동 (탱크 턴)

    // ===> 주의: setMotorSpeedDifferential 함수 로직과 방향 일치 확인 필요 <===

    int leftSpeed  = -correction; // correction > 0 (오른쪽) -> 왼쪽 후진

    int rightSpeed =  correction; // correction > 0 (오른쪽) -> 오른쪽 전진

    setMotorSpeedDifferential(leftSpeed, rightSpeed);



    delay(10); // 제어 루프 주기 (약 100Hz)

  } // end while

}



// ===== 명령 파싱/입력 =====

bool parseCommand(const char* command, int &speed, float &angle, int &s_param) {

  // 'v', 'a', 's' 문자가 모두 있는지 확인

  const char* v_ptr = strchr(command, 'v');

  const char* a_ptr = strchr(command, 'a');

  const char* s_ptr = strchr(command, 's');

  if (!v_ptr || !a_ptr || !s_ptr) return false; // 하나라도 없으면 실패



  // 각 문자 다음의 숫자를 파싱

  speed   = atoi(v_ptr + 1);

  angle   = atof(a_ptr + 1); // float로 파싱

  s_param = atoi(s_ptr + 1);

  return true; // 성공

}



// 지정된 시리얼 포트(Stream)로부터 명령 한 줄 읽고 처리

void handleInputFrom(Stream &port) {

  while (port.available()) {

    char incomingByte = port.read();



    // mySerial(핀 6,7)로 받은 데이터는 USB 시리얼로 다시 보내줌 (디버깅용)

    if (&port == &mySerial) {

      Serial.write(incomingByte);

    }



    // ----- Zero 단축키: 'z' 또는 'Z' 처리 -----

    if (incomingByte == 'z' || incomingByte == 'Z') {

      imuUpdate(); updateHeadingFused(); // 센서값 최신화

      yawZeroOffset = lastOutYaw;      // 현재 내부 Yaw 값을 오프셋으로 저장

      initialHeading = currentHeading; // 현재 헤딩(거의 0)을 새 기준으로 설정

      targetAngle    = initialHeading; // 목표 각도도 리셋

      Serial.println("[ZERO] heading set to 0 deg"); // PC로 메시지 출력

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

          targetAngle = wrap360(initialHeading + receivedAngle); // 목표 절대 각도 계산



          // 디버그 메시지 출력

          if (DEBUG) {

            Serial.print("[CMD] v="); Serial.print(targetSpeed);

            Serial.print(" a_rel=");  Serial.print(receivedAngle, 1);

            Serial.print(" -> tgtAbs="); Serial.println(targetAngle, 1);

          }



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



// 양쪽 시리얼 포트(mySerial, Serial) 모두 확인

void handleUartInput() {

  // SoftwareSerial 사용 시 listen() 필수

  mySerial.listen();

  handleInputFrom(mySerial); // 핀 6, 7 확인



  // USB 시리얼 확인 (listen 불필요)

  handleInputFrom(Serial);   // USB 연결 확인

}



// ===== Arduino 기본 Setup =====

void setup() {

  Serial.begin(UART_BAUD_RATE);     // PC 모니터용 (115200)

  mySerial.begin(115200);           // 명령 수신용 (115200)

  imuSetup();                       // WT901 센서용 (9600)



  // 모터 핀 출력 설정

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);



  // 시작 시 잠깐 움직여서 모터 풀어주기

  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);

  delay(500);



  // ===== 초기 헤딩 설정 (안정화 대기) =====

  unsigned long t0 = millis();

  uint16_t angSamples = 0;

  float lastYawSample = 9999.0f;

  Serial.print("Waiting for stable IMU angle data...");

  while (millis() - t0 < 2000) {  // 최대 2초 대기

    imuUpdate(); // 센서 읽기

    updateHeadingFused(); // 헤딩 계산

    // Yaw 값이 안정화 (최소 5번 다른 값 수신)될 때까지 기다림

    if (stcAngle.Yaw != lastYawSample) {

      lastYawSample = stcAngle.Yaw;

      angSamples++;

      Serial.print(".");

      if (angSamples >= 5) break; // 안정화 완료

    }

    delay(10);

  }

  Serial.println(" OK.");



  // 현재 방향을 0도로 설정 (선택 사항)

  yawZeroOffset = lastOutYaw;      // 현재 내부 Yaw 값을 오프셋으로 사용

  updateHeadingFused();            // currentHeading을 0 근처로 갱신



  initialHeading = currentHeading; // 현재 각도(0 근처)를 초기 기준으로 설정

  targetAngle    = initialHeading;



  Serial.println("Setup complete. Ready for commands.");

}



// ===== Arduino 기본 Loop =====

void loop() {

  // 1. 명령 수신 및 상태 변경

  handleUartInput();



  // 2. 현재 상태에 따른 동작 수행

  switch (currentState) {

    case IDLE:

      // 할 일 없음, 명령 대기

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

          Serial.println("Rotation Timeout!"); // 오류 메시지

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



  // 4. 디버그 메시지 주기적 출력 (0.2초마다)

  if (DEBUG && millis() - lastDbgMs >= DBG_PERIOD_MS) {

    lastDbgMs = millis();

    float errDbg = angleDiff(targetAngle, currentHeading);

    dbgPrintStateLine("[HDG]", errDbg);

  }



  // 5. IMU 프레임 수신 카운터 출력 (1초마다)

  if (millis() - lastCountMs >= 1000) {

    lastCountMs = millis();

    Serial.print("[CNT] acc="); Serial.print(accCnt);

    Serial.print(" gyr="); Serial.print(gyrCnt);

    Serial.print(" ang="); Serial.println(angCnt);

    accCnt = gyrCnt = angCnt = 0; // 카운터 리셋

  }



  // 6. 메인 루프 지연 (약 50Hz)

  delay(20);

}
