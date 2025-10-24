#include <Wire.h>
#include "I2Cdev.h" // MPU950 라이브러리 의존성
#include "MPU9250.h" // MPU950 자이로 센서 라이브러리
// #include <SoftwareSerial.h> // Uno/Nano 등 Serial1이 없는 보드 사용 시 주석 해제

// =========================================================================
// ==                          전역 변수 및 상수                          ==
// =========================================================================

// ----- MPU9250 센서 관련 -----
MPU9250 mpu;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz; // 원시 센서 값
float Axyz[3], Gxyz[3], Mxyz[3];           // 처리된 센서 값
float mx_centre = 0.0, my_centre = 0.0, mz_centre = 0.0; // 지자기 오프셋
float currentHeading = 0.0;                 // 현재 절대 헤딩
float initialHeading = 0.0;                 // 기준 헤딩 (초기 0도)

// ----- 모터 드라이버 핀 -----
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 11
#define IN3 12
#define IN4 13

// ----- PID 제어 상수 -----
float Kp = 15.0;
float Ki = 0.01;
float Kd = 1.0;

// ----- 모터 제어 파라미터 -----
const int MIN_PWM = 40;
const int MAX_PWM = 100;         // 회전 시 최대 PWM
const int MAX_MOVE_PWM = 150;    // 직진/후진 시 최대 PWM
const int INITIAL_MOVE_SPEED = 50;
const int INITIAL_MOVE_DURATION = 500; // ms
const unsigned long MOVE_DURATION = 2000; // ms

// ----- 목표 값 및 상태 변수 -----
float targetAngle = 0.0;         // 목표 절대 각도
int   targetSpeed = 0;         // 목표 속도
bool  newCommandReceived = false;// 새 명령 수신 플래그
enum RobotState { IDLE, ROTATING, MOVING }; // 로봇 상태
RobotState currentState = IDLE;
unsigned long moveStartTime = 0;

// ----- UART 통신 설정 -----
#define UART_BAUD_RATE 9600
// SoftwareSerial mySerial(10, 11); // Uno/Nano 사용 시

// =========================================================================
// ==                          유틸리티 함수                             ==
// =========================================================================

// 각도를 0 ~ 360 범위로 정규화
static inline float wrap360(float angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

// 두 각도 사이의 최단 차이 계산 (-180 ~ +180)
float angleDiff(float target, float current) {
  float diff = wrap360(target) - wrap360(current);
  if (diff >= 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

// =========================================================================
// ==                          모터 제어 함수                             ==
// =========================================================================

// 단일 모터 제어 (속도: -255 ~ 255)
void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { // 전진
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, speed);
  } else if (speed < 0) { // 후진
    digitalWrite(IN1_pin, LOW); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, -speed);
  } else { // 정지 (Brake)
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, 0);
  }
}

// 차동 구동 설정 (회전용)
void setMotorSpeedDifferential(int leftSpeed, int rightSpeed) {
   leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
   rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
   int actualLeftSpeed = 0, actualRightSpeed = 0;
   int turnPWM = 0;

   if (rightSpeed > leftSpeed) { // 시계방향
       turnPWM = constrain(rightSpeed, MIN_PWM, MAX_PWM);
       actualLeftSpeed = -turnPWM; actualRightSpeed = turnPWM;
   } else if (leftSpeed > rightSpeed) { // 반시계방향
       turnPWM = constrain(leftSpeed, MIN_PWM, MAX_PWM);
       actualLeftSpeed = turnPWM; actualRightSpeed = -turnPWM;
   }
   driveOneMotor(ENA, IN1, IN2, actualLeftSpeed);
   driveOneMotor(ENB, IN3, IN4, actualRightSpeed);
}

// 직진/후진 함수
void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  driveOneMotor(ENA, IN1, IN2, speed);
  driveOneMotor(ENB, IN3, IN4, speed);
}

// =========================================================================
// ==                        센서 처리 및 각도 계산 함수                   ==
// =========================================================================

// 원시 가속도 데이터 읽고 g 단위 변환
void getAccel_Data(void) {
    mpu.getAcceleration(&ax, &ay, &az);
    float accelSensitivity = 8192.0; // +/- 4g 기준 (imuSetup과 일치 확인!)
    Axyz[0] = (float)ax / accelSensitivity;
    Axyz[1] = (float)ay / accelSensitivity;
    Axyz[2] = (float)az / accelSensitivity;
}

// 원시 자이로 데이터 읽고 deg/s 단위 변환
void getGyro_Data(void) {
    mpu.getRotation(&gx, &gy, &gz);
    float gyroSensitivity = 65.5; // +/- 500 dps 기준 (imuSetup과 일치 확인!)
    Gxyz[0] = (float)gx / gyroSensitivity;
    Gxyz[1] = (float)gy / gyroSensitivity;
    Gxyz[2] = (float)gz / gyroSensitivity;
}

// 원시 지자기 데이터 읽기
void getCompass_Data_Raw(void) {
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
}

// 지자기 데이터에 칼리브레이션 오프셋 적용
void applyCompassCalibration() {
    Mxyz[0] = (float)mx - mx_centre;
    Mxyz[1] = (float)my - my_centre;
    Mxyz[2] = (float)mz - mz_centre;
}

// 기울기 보정 각도 계산
void calculateTiltHeading(void) {
    float roll_rad = atan2(Axyz[1], Axyz[2]);
    float pitch_rad = atan2(-Axyz[0], sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]));
    float cos_roll = cos(roll_rad), sin_roll = sin(roll_rad);
    float cos_pitch = cos(pitch_rad), sin_pitch = sin(pitch_rad);
    float mag_x_horiz = Mxyz[0] * cos_pitch + Mxyz[1] * sin_roll * sin_pitch + Mxyz[2] * cos_roll * sin_pitch;
    float mag_y_horiz = Mxyz[1] * cos_roll - Mxyz[2] * sin_roll;
    float heading_rad = atan2(-mag_y_horiz, mag_x_horiz);
    currentHeading = wrap360(heading_rad * 180.0 / PI);
}

// MPU9250 초기화
void imuSetup() {
  Wire.begin();
  Serial.println("I2C 장치 초기화 중...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU9250 연결 성공");
    mpu.setFullScaleGyroRange(MPU9250_GYRO_FS_500); // 자이로 +/- 500 dps
    mpu.setFullScaleAccelRange(MPU9250_ACCEL_FS_4);   // 가속도 +/- 4g
    mpu.setDLPFMode(MPU9250_DLPF_BW_42);
    Serial.println("주의: 가속도/자이로 오프셋 적용 코드가 필요합니다!");
    // --- 가속도/자이로 오프셋 적용 위치 ---
  } else {
    Serial.println("MPU9250 연결 실패 - 실행 중지");
    while (1);
  }
  Serial.println("IMU 설정 완료.");
}

// 지자기 센서 칼리브레이션 함수
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
        getCompass_Data_Raw();
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
    delay(1000);
}

// 모든 센서 데이터 업데이트 및 최종 헤딩 계산
void imuUpdate() {
  getAccel_Data();          // 가속도 읽고 처리 (Axyz)
  getCompass_Data_Raw();    // 원시 지자기 읽기 (mx, my, mz)
  applyCompassCalibration(); // 지자기 오프셋 적용 (Mxyz)
  calculateTiltHeading();   // 최종 헤딩 계산 (currentHeading)
}

// =========================================================================
// ==                          PID 제어 및 회전 함수                       ==
// =========================================================================

// 목표 '절대 각도'로 회전하는 함수
bool rotateToAbsAngle(float targetAbsAngle) {
  float currentAbsAngle = 0.0, error = 0.0;
  float integral = 0.0, lastError = 0.0, derivative = 0.0;
  int correction = 0;
  const float tolerance = 2.0; // 도
  unsigned long lastTime = micros(), startTime = millis();
  const unsigned long TIMEOUT_DURATION = 15000; // ms

  Serial.print("회전 시작: 목표 절대 각도 "); Serial.println(targetAbsAngle);

  do {
    if (millis() - startTime > TIMEOUT_DURATION) { // 타임아웃
        Serial.println("오류: 회전 시간 초과!");
        setMotorSpeedDifferential(0, 0); return false;
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0; // 초
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01;

    imuUpdate(); // 현재 각도 갱신
    currentAbsAngle = currentHeading;
    error = angleDiff(targetAbsAngle, currentAbsAngle);

    // PID 계산
    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = round(Kp * error + Ki * integral + Kd * derivative);

    // 모터 속도 결정
    int leftSpeed  = -correction;
    int rightSpeed = correction;
    setMotorSpeedDifferential(leftSpeed, rightSpeed); // 회전 실행

    // 디버깅 출력 (USB)
    Serial.print(" TargetAbs:"); Serial.print(targetAbsAngle, 1);
    Serial.print(" CurrAbs:"); Serial.print(currentAbsAngle, 1);
    Serial.print(" Err:"); Serial.print(error, 1);
    Serial.print(" Corr:"); Serial.println(correction);

    delay(10); // 제어 루프 주기

  } while (abs(error) > tolerance);

  setMotorSpeedDifferential(0, 0); // 목표 도달 시 정지
  Serial.println("목표 각도 도달!");
  return true; // 성공
}

// =========================================================================
// ==                      통신 및 명령어 처리 함수                       ==
// =========================================================================

// "v<speed> a<angle>" 형식 파싱
bool parseCommand(String command, int &speed, float &angle, int &s_param) {
  command.toLowerCase(); // 모든 문자를 소문자로 변경

  // v, a, s 마커가 모두 존재하는지 확인
  int v_index = command.indexOf('v');
  int a_index = command.indexOf('a');
  int s_index = command.indexOf('s');

  if (v_index == -1 || a_index == -1 || s_index == -1) {
    return false; // 필수 마커 중 하나라도 없으면 실패
  }

  // String 객체를 C 스타일 문자열 포인터로 변환하여 파싱
  // atoi/atof는 숫자가 아닌 문자를 만나면 자동으로 파싱을 중단합니다.
  const char* cmd_str = command.c_str();
  speed   = atoi(cmd_str + v_index + 1);
  angle   = atof(cmd_str + a_index + 1);
  s_param = atoi(cmd_str + s_index + 1);
  
  return true;
}

// UART 입력 처리 함수
void handleUartInput() {
  if (Serial1.available() > 0) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    Serial.print("UART 명령 수신: "); Serial.println(input);

    int receivedSpeed = 0;
    float receivedAngle = 0.0;
    int receivedS = 0; // 's' 파라미터를 저장할 변수 추가

    // 새로 만든 parseCommand 함수 호출
    if (parseCommand(input, receivedSpeed, receivedAngle, receivedS)) {
      targetSpeed = constrain(receivedSpeed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
      targetAngle = wrap360(initialHeading + receivedAngle);
      // 여기서 receivedS 변수를 사용하면 됩니다. 예:
      // targetS = receivedS; 

      newCommandReceived = true;

      // 파싱 결과 전체를 USB 시리얼 모니터에 출력
      Serial.print("파싱 결과 -> V: "); Serial.print(targetSpeed);
      Serial.print(", A: "); Serial.print(targetAngle);
      Serial.print(", S: "); Serial.println(receivedS); // S 값 출력 추가

      // 현재 동작 중단
      if (currentState == MOVING || currentState == ROTATING) {
          setMotorSpeedDifferential(0, 0);
          moveAtSpeed(0);
          Serial.println("현재 동작 중단.");
      }
      currentState = ROTATING;

    } else {
      Serial.println("오류: 명령어 형식 불량 (예: v500a203s10)");
    }
  }
}

// =========================================================================
// ==                          메인 설정 및 루프                           ==
// =========================================================================

void setup() {
  Serial.begin(115200);      // USB 시리얼 (디버깅)
  Serial1.begin(UART_BAUD_RATE); // UART 통신
  // mySerial.begin(UART_BAUD_RATE); // SoftwareSerial 사용 시

  Serial.println("=== 차량 제어 시작 (UART 명령 대기) ===");

  // 모터 핀 초기화
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.println("모터 핀 초기화 완료.");

  imuSetup(); // IMU 초기화 및 설정

  // --- 지자기 센서 칼리브레이션 ---
  runCompassCalibration(); // 전원 켤 때마다 실행 (USB로 'ready' 입력 필요)
  // --- 또는 저장된 값 사용 ---
  // mx_centre = ...; my_centre = ...; mz_centre = ...;
  // Serial.println("저장된 지자기 오프셋 적용.");

  // --- 초기 직진 및 기준 각도 설정 ---
  Serial.println("초기 위치 조정을 위해 잠시 직진...");
  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);
  Serial.println("초기 직진 완료. 각도 기준 설정 대기...");
  delay(1000); // 안정화
  Serial.println("현재 헤딩(방향각)을 기준으로 설정...");
  for (int i = 0; i < 10; i++) { imuUpdate(); delay(50); }
  initialHeading = currentHeading;
  targetAngle = initialHeading;
  Serial.print("초기 헤딩 기준 설정 완료: "); Serial.println(initialHeading, 2);

  Serial.println("설정 완료. 아두이노 B로부터 UART 명령 대기 중...");
  currentState = IDLE; // 초기 상태: 대기
}

void loop() {
  handleUartInput(); // UART 입력 처리

  // --- 상태 머신 실행 ---
  switch (currentState) {
    case IDLE:
      // 새 명령 대기
      break;

    case ROTATING:
      if (newCommandReceived) {
          newCommandReceived = false; // 플래그 리셋
          bool rotationSuccess = rotateToAbsAngle(targetAngle); // 회전 시도
          if (rotationSuccess) {
              if (targetSpeed != 0) { // 이동 필요
                  currentState = MOVING;
                  moveStartTime = millis();
                  Serial.print("목표 속도 "); Serial.print(targetSpeed); Serial.println(" 로 이동 시작.");
                  moveAtSpeed(targetSpeed); // 이동 시작
              } else { // 이동 불필요 (속도 0)
                  currentState = IDLE;
                  Serial.println("목표 속도가 0이므로 대기합니다.");
              }
          } else { // 회전 실패
              currentState = IDLE;
              Serial.println("회전 실패. 대기 상태로 복귀합니다.");
          }
      }
      break;

    case MOVING:
      // 지정된 시간(MOVE_DURATION) 동안 이동 후 정지
      if (millis() - moveStartTime >= MOVE_DURATION) {
          Serial.println("이동 시간 완료. 정지합니다.");
          moveAtSpeed(0); // 정지
          currentState = IDLE; // 대기 상태로 전환
          Serial.println("다음 UART 명령 대기 중...");
      }
      // 이동 중 새 명령 수신 시 handleUartInput()에서 처리됨
      break;
  }

  delay(50); // 메인 루프 지연
}
