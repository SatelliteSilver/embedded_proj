#include <Wire.h>
#include "I2Cdev.h" // MPU9250 라이브러리 의존성
#include "MPU9250.h" // MPU9250 자이로 센서 라이브러리
#include <SoftwareSerial.h>
SoftwareSerial mySerial(6, 7); // 아두이노 B(제어 보드)와 통신용
// =========================================================================
// ==                           전역 변수 및 상수                           ==
// =========================================================================

// ----- MPU9250 센서 관련 -----
MPU9250 mpu;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
float Axyz[3], Gxyz[3], Mxyz[3];
float mx_centre = 382.5, my_centre = 139.0, mz_centre = 120.0; // 캘리브레이션 값 (반드시 재수행 필요)
float currentHeading = 244.9; // setup()에서 덮어쓰므로 초기값 무의미
float initialHeading = 0.0;

// ----- 모터 드라이버 핀 -----
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 11
#define IN3 12
#define IN4 13

// ----- PID 제어 상수 [!! 튜닝 필요 !!] -----
// [수정] 모터 흔들림(진동) 현상을 잡기 위해 튜닝 시작 값으로 변경합니다.
// Kp가 15.0이면 너무 높아 오버슈트가 발생할 가능성이 큽니다.
float Kp = 4.0;  // 15.0 -> 4.0 (값을 낮춰서 시작)
float Ki = 0.0;  // 0.01 -> 0.0 (튜닝을 위해 0에서 시작)
float Kd = 0.1;  // 1.0 -> 0.1 (값을 낮춰서 시작)

// ----- 모터 제어 파라미터 -----
const int MIN_PWM = 40;       // 제자리 회전 시 최소 PWM
const int MAX_PWM = 250;      // 최대 PWM
const int MAX_MOVE_PWM = 150; // 직진 주행 시 최대 PWM
const int INITIAL_MOVE_SPEED = 50;
const int INITIAL_MOVE_DURATION = 500; // ms
const unsigned long MOVE_DURATION = 2000; // ms

// ----- 목표 값 및 상태 변수 -----
float targetAngle = 0.0;
int   targetSpeed = 0;
bool  newCommandReceived = false;
enum RobotState { IDLE, ROTATING, MOVING };
RobotState currentState = IDLE;
unsigned long moveStartTime = 0;

// ----- UART 통신 설정 -----
#define UART_BAUD_RATE 115200 // USB 시리얼 모니터용
// mySerial(6, 7)은 115200으로 setup()에서 설정됨

#define UART_BUFFER_SIZE 32
char uartBuffer[UART_BUFFER_SIZE];
byte uartBufferIndex = 0;


// =========================================================================
// ==                           유틸리티 함수                             ==
// =========================================================================

/**
 * @brief 각도를 0.0 ~ 359.9... 범위로 정규화합니다.
 */
static inline float wrap360(float angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

/**
 * @brief 두 각도 사이의 최단 차이(-180 ~ +180)를 계산합니다.
 */
float angleDiff(float target, float current) {
  float diff = wrap360(target) - wrap360(current);
  if (diff >= 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

// =========================================================================
// ==                           모터 제어 함수                            ==
// =========================================================================

/**
 * @brief 모터 1개를 제어합니다.
 * @param speed -255(역회전) ~ +255(정회전). 0은 정지.
 */
void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, speed);
  } else if (speed < 0) {
    digitalWrite(IN1_pin, LOW); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, -speed);
  } else {
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, HIGH); // 브레이크
    analogWrite(EN_pin, 0);
  }
}

/**
 * @brief 제자리 회전을 위한 차동 구동 설정
 * @param leftSpeed -MAX_PWM ~ +MAX_PWM
 * @param rightSpeed -MAX_PWM ~ +MAX_PWM
 */
void setMotorSpeedDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  
  int actualLeftSpeed = 0, actualRightSpeed = 0;
  int turnPWM = 0;

  // 이 로직은 한쪽이 +이면 다른 쪽은 -가 되도록 하여 제자리 회전을 유도합니다.
  if (rightSpeed > leftSpeed) { // 우회전 (leftSpeed < 0, rightSpeed > 0)
    turnPWM = constrain(rightSpeed, MIN_PWM, MAX_PWM);
    actualLeftSpeed = -turnPWM; actualRightSpeed = turnPWM;
  } else if (leftSpeed > rightSpeed) { // 좌회전 (leftSpeed > 0, rightSpeed < 0)
    turnPWM = constrain(leftSpeed, MIN_PWM, MAX_PWM);
    actualLeftSpeed = turnPWM; actualRightSpeed = -turnPWM;
  }
  // 속도가 0이면 driveOneMotor(0)에 의해 정지됩니다.
  driveOneMotor(ENA, IN1, IN2, actualLeftSpeed);
  driveOneMotor(ENB, IN3, IN4, actualRightSpeed);
}

/**
 * @brief 두 모터를 같은 속도로 구동 (직진/후진)
 * @param speed -MAX_MOVE_PWM ~ +MAX_MOVE_PWM
 */
void moveAtSpeed(int speed) {
  // [참고] 후진을 막는 로직은 handleUartInput()에 있으므로
  // 이 함수 자체는 후진(음수)도 가능하도록 둡니다. (초기화 동작 등에서 사용 가능)
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  driveOneMotor(ENA, IN1, IN2, speed);
  driveOneMotor(ENB, IN3, IN4, speed);
}

// =========================================================================
// ==                       센서 처리 및 각도 계산 함수                     ==
// =========================================================================

void getAccel_Data(void) {
  mpu.getAcceleration(&ax, &ay, &az);
  float accelSensitivity = 8192.0; // FS=4G (setFullScaleAccelRange)
  Axyz[0] = (float)ax / accelSensitivity;
  Axyz[1] = (float)ay / accelSensitivity;
  Axyz[2] = (float)az / accelSensitivity;
}

void getGyro_Data(void) {
  mpu.getRotation(&gx, &gy, &gz);
  float gyroSensitivity = 65.5; // FS=500DPS (setFullScaleGyroRange)
  Gxyz[0] = (float)gx / gyroSensitivity;
  Gxyz[1] = (float)gy / gyroSensitivity;
  Gxyz[2] = (float)gz / gyroSensitivity;
}

void getCompass_Data_Raw(void) {
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
}

void applyCompassCalibration() {
  Mxyz[0] = (float)mx - mx_centre;
  Mxyz[1] = (float)my - my_centre;
  Mxyz[2] = (float)mz - mz_centre;
}

void calculateTiltHeading(void) {
  float roll_rad = atan2(Axyz[1], Axyz[2]);
  float pitch_rad = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]));
  float cos_roll = cos(roll_rad), sin_roll = sin(roll_rad);
  float cos_pitch = cos(pitch_rad), sin_pitch = sin(pitch_rad);
  
  // 틸트 보정 (수평면 자기장 계산)
  float mag_x_horiz = Mxyz[0] * cos_pitch + Mxyz[1] * sin_roll * sin_pitch + Mxyz[2] * cos_roll * sin_pitch;
  float mag_y_horiz = Mxyz[1] * cos_roll - Mxyz[2] * sin_roll;
  
  float heading_rad = atan2(-mag_y_horiz, mag_x_horiz);
  currentHeading = wrap360(heading_rad * 180.0 / PI);
}

void imuSetup() {
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    mpu.setFullScaleGyroRange(MPU9250_GYRO_FS_500);
    mpu.setFullScaleAccelRange(MPU9250_ACCEL_FS_4);
    mpu.setDLPFMode(MPU9250_DLPF_BW_42); // 저역 통과 필터
  } else {
    // 연결 실패 시 정지
    while (1); 
  }
}

// [!! 주의 !!] 이 함수는 USB Serial(0, 1핀)로만 동작합니다.
// 아두이노 B를 0, 1핀에 연결하면 이 함수를 사용할 수 없습니다.
void runCompassCalibration() {
  Serial.println(F("=== 지자기 센서 칼리브레이션 ==="));
  Serial.println(F("20초 동안 센서를 모든 방향으로 천천히 회전시켜 주세요."));
  Serial.println(F("시작하려면 시리얼 모니터(USB)에 'ready'를 입력하세요."));
  while (!Serial.find((char*)"ready"));
  Serial.println(F("칼리브레이션 시작... 20초간 회전하세요."));

  long startTime = millis(); long duration = 20000;
  int16_t mx_max_cal = -32767, my_max_cal = -32767, mz_max_cal = -32767;
  int16_t mx_min_cal = 32767, my_min_cal = 32767, mz_min_cal = 32767;

  while (millis() - startTime < duration) {
    getCompass_Data_Raw(); // 가속도/자이로도 같이 읽지만 지자기(mx,my,mz)만 사용
    if (mx > mx_max_cal) mx_max_cal = mx; if (mx < mx_min_cal) mx_min_cal = mx;
    if (my > my_max_cal) my_max_cal = my; if (my < my_min_cal) my_min_cal = my;
    if (mz > mz_max_cal) mz_max_cal = mz; if (mz < mz_min_cal) mz_min_cal = mz;
    delay(20);
  }
  mx_centre = (float)(mx_max_cal + mx_min_cal) / 2.0;
  my_centre = (float)(my_max_cal + my_min_cal) / 2.0;
  mz_centre = (float)(mz_max_cal + mz_min_cal) / 2.0;

  Serial.println(F("칼리브레이션 완료!"));
  Serial.print(F("계산된 중심 오프셋 (X, Y, Z): "));
  Serial.print(mx_centre); Serial.print(F(", "));
  Serial.print(my_centre); Serial.print(F(", ")); Serial.println(mz_centre);
  Serial.println(F("============================="));
  Serial.println(F("이 값들을 코드 상단의 전역 변수 값에 복사하세요."));
  delay(1000);
}

void imuUpdate() {
  getAccel_Data(); // 가속도 값 (Axyz) 업데이트
  getCompass_Data_Raw(); // mx, my, mz 값 업데이트
  applyCompassCalibration(); // Mxyz 값 업데이트
  calculateTiltHeading(); // currentHeading 값 업데이트
}

// =========================================================================
// ==                       PID 제어 및 회전 함수                         ==
// =========================================================================

/**
 * @brief PID 제어를 사용하여 목표 절대 각도로 회전합니다.
 * @param targetAbsAngle 목표 절대 각도 (0~360)
 * @return true: 성공, false: 타임아웃
 */
bool rotateToAbsAngle(float targetAbsAngle) {
  float currentAbsAngle = 0.0, error = 0.0;
  float integral = 0.0, lastError = 0.0, derivative = 0.0;
  int correction = 0;
  
  const float tolerance = 2.0; // 목표 각도 허용 오차 (도)
  unsigned long lastTime = micros(), startTime = millis();
  const unsigned long TIMEOUT_DURATION = 15000; // 15초 타임아웃

  do {
    // 타임아웃 검사
    if (millis() - startTime > TIMEOUT_DURATION) {
      setMotorSpeedDifferential(0, 0); // 모터 정지
      return false; // 실패 반환
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0; // 초 단위 시간 변화
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01; // 루프가 너무 빠를 경우 대비

    imuUpdate(); // 현재 각도(currentHeading) 업데이트
    currentAbsAngle = currentHeading;
    error = angleDiff(targetAbsAngle, currentAbsAngle); // 목표-현재 최단 각도

    // PID 계산
    integral += error * deltaTime; // 오차 적분
    // Ki가 0이 아니면 integral 값이 무한히 커지는 것을 방지(Integral Windup)하는 로직이 필요할 수 있습니다.
    
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0; // 오차 미분
    lastError = error;

    correction = round(Kp * error + Ki * integral + Kd * derivative);

    // PID 출력을 모터 속도로 변환
    int leftSpeed  = -correction;
    int rightSpeed = correction;
    
    // setMotorSpeedDifferential 함수가 MIN_PWM ~ MAX_PWM 사이로 속도를 조절해줍니다.
    setMotorSpeedDifferential(leftSpeed, rightSpeed);

    delay(10); // 제어 루프 주기

  } while (abs(error) > tolerance); // 오차가 허용 범위 밖인 동안 반복

  setMotorSpeedDifferential(0, 0); // 목표 도달 시 모터 정지
  return true; // 성공 반환
}

// =========================================================================
// ==                       통신 및 명령어 처리 함수                      ==
// =========================================================================

/**
 * @brief "v...a...s..." 형식의 문자열을 파싱합니다.
 * @return true: 파싱 성공, false: 형식 오류
 */
bool parseCommand(const char* command, int &speed, float &angle, int &s_param) {
  const char* v_ptr = strchr(command, 'v');
  const char* a_ptr = strchr(command, 'a');
  const char* s_ptr = strchr(command, 's');

  if (v_ptr == NULL || a_ptr == NULL || s_ptr == NULL) {
    return false; // 'v', 'a', 's' 중 하나라도 없으면 실패
  }

  // 각 포인터 다음 글자부터 숫자로 변환
  speed   = atoi(v_ptr + 1);
  angle   = atof(a_ptr + 1);
  s_param = atoi(s_ptr + 1);

  return true;
}


/**
 * @brief mySerial(SoftwareSerial) 포트로 완료 신호를 전송합니다.
 */
void sendCompletionSignal() {
  imuUpdate(); // 최종 각도 업데이트
  float finalRelativeAngle = angleDiff(currentHeading, initialHeading);

  mySerial.print('a');
  mySerial.print(finalRelativeAngle, 1); // 소수점 1자리
  mySerial.println("s0"); // println이 \r\n을 붙여줌
}

/**
 * @brief mySerial(SoftwareSerial) 포트에서 입력을 처리합니다.
 */
void handleUartInput() {
  
  if (mySerial.available()) {
    char incomingByte = mySerial.read();
    
    // [디버깅용] 아두이노 B(제어보드)로부터 받은 데이터를 USB로 그대로 출력
    Serial.write(incomingByte); 

    if (incomingByte == '\n' || incomingByte == '\r') { // 명령어 종료 문자 수신
      if (uartBufferIndex > 0) { // 버퍼에 데이터가 있다면
        uartBuffer[uartBufferIndex] = '\0'; // 문자열 종료

        int receivedSpeed;
        float receivedAngle;
        int receivedS;

        if (parseCommand(uartBuffer, receivedSpeed, receivedAngle, receivedS)) {
          // 파싱 성공
          
          // [수정] 후진(음수 속도) 방지. 0 ~ MAX_MOVE_PWM 사이로만 속도를 받음.
          targetSpeed = constrain(receivedSpeed, 0, MAX_MOVE_PWM);
          
          // [수정] 오타 수정 (wrap380 -> wrap360)
          targetAngle = wrap360(initialHeading + receivedAngle); // 상대 각도를 절대 각도로 변환
          
          newCommandReceived = true;

          // 만약 이전 동작(이동 또는 회전) 중이었다면 즉시 정지
          if (currentState == MOVING || currentState == ROTATING) {
            setMotorSpeedDifferential(0, 0);
            moveAtSpeed(0);
          }
          currentState = ROTATING; // 새 명령을 받았으므로 '회전' 상태로 전환
        } else {
          // 명령어 형식 오류 (무시)
        }
      }
      uartBufferIndex = 0; // 버퍼 인덱스 초기화

    } else if (uartBufferIndex < (UART_BUFFER_SIZE - 1)) {
      // 버퍼에 바이트 추가
      uartBuffer[uartBufferIndex++] = incomingByte;
    }
    // 버퍼가 꽉 찼는데 종료 문자가 안 들어오면 다음 '\n'까지 데이터 유실 (정상)
  }
}


// =========================================================================
// ==                           메인 설정 및 루프                           ==
// =========================================================================

void setup() {
  // USB 시리얼: 디버깅 및 캘리브레이션용 (115200)
  Serial.begin(UART_BAUD_RATE); 
  
  // Software Serial: 아두이노 B(제어보드)와 통신용 (115200)
  // [주의] 115200은 SoftwareSerial에 다소 불안정할 수 있습니다.
  // 통신 오류가 발생하면 57600 또는 9600으로 낮추는 것을 고려하세요.
  mySerial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  imuSetup(); // MPU9250 초기화
  
  // [!! 중요 !!]
  // 캘리브레이션은 아두이노 B를 연결하기 전에(핀 6, 7 사용 전)
  // USB(Serial)만 연결하고, 아래 주석을 해제하여 1회 실행해야 합니다.
  // runCompassCalibration(); 
  
  // 초기화 동작: 잠깐 전진했다가 멈춤
  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);
  delay(1000); // 센서 안정화 대기
  
  // 초기 헤딩 값 설정
  for (int i = 0; i < 10; i++) { imuUpdate(); delay(50); } // 평균을 위해 여러 번 읽기
  initialHeading = currentHeading;
  targetAngle = initialHeading; // 현재 각도를 목표 각도로 설정

  currentState = IDLE; // 대기 상태로 시작
  Serial.println("Setup complete. Ready for commands.");
}

void loop() {

  handleUartInput(); // 항상 SoftwareSerial 입력 확인

  switch (currentState) {
    case IDLE:
      // 새 명령을 기다리며 대기
      break;

    case ROTATING:
      if (newCommandReceived) {
        newCommandReceived = false; // 명령 처리 시작
        
        bool rotationSuccess = rotateToAbsAngle(targetAngle);
        
        if (rotationSuccess) {
          if (targetSpeed != 0) { // 직진 속도 명령이 있었다면
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(targetSpeed); // 직진 시작
          } else { // 제자리 회전만(speed 0) 하는 명령이었다면
            sendCompletionSignal(); // mySerial로 완료 신호 전송
            currentState = IDLE; // 대기 상태로 복귀
          }
        } else { // 회전 타임아웃
          sendCompletionSignal(); // 실패했어도 완료 신호 전송
          currentState = IDLE;
        }
      }
      break;

    case MOVING:
      // MOVE_DURATION 동안 직진
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0); // 시간 다 되면 정지
        sendCompletionSignal(); // mySerial로 완료 신호 전송
        currentState = IDLE; // 대기 상태로 복귀
      }
      break;
  }

  // [디버깅용] 현재 상태를 USB 시리얼 모니터로 출력
  // Serial.print("Heading: "); Serial.print(currentHeading);
  // Serial.print(" Target: "); Serial.print(targetAngle);
  // Serial.print(" State: "); Serial.println(currentState);

  delay(50); // 메인 루프 안정화
}
