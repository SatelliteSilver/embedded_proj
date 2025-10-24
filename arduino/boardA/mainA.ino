#include <Wire.h>
#include "I2Cdev.h" // MPU9250 라이브러리 의존성
#include "MPU9250.h" // MPU9250 자이로 센서 라이브러리
// SoftwareSerial 라이브러리 삭제
#include <SoftwareSerial.h>
SoftwareSerial mySerial(6, 7); 
// =========================================================================
// ==                           전역 변수 및 상수                           ==
// =========================================================================

// ----- MPU9250 센서 관련 -----
MPU9250 mpu;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
float Axyz[3], Gxyz[3], Mxyz[3];
float mx_centre = 382.5, my_centre = 139.0, mz_centre = 120.0; // 캘리브레이션 값
float currentHeading = 244.9; // setup()에서 덮어쓰므로 초기값 무의미
float initialHeading = 0.0;

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
const int MAX_PWM = 250;
const int MAX_MOVE_PWM = 150;
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
#define UART_BAUD_RATE 115200
// mySerial 객체 삭제

#define UART_BUFFER_SIZE 32
char uartBuffer[UART_BUFFER_SIZE];
byte uartBufferIndex = 0;


// =========================================================================
// ==                           유틸리티 함수                             ==
// =========================================================================

static inline float wrap360(float angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

float angleDiff(float target, float current) {
  float diff = wrap360(target) - wrap360(current);
  if (diff >= 180.0)  diff -= 360.0;
  if (diff < -180.0) diff += 360.0;
  return diff;
}

// =========================================================================
// ==                           모터 제어 함수                            ==
// =========================================================================

void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, speed);
  } else if (speed < 0) {
    digitalWrite(IN1_pin, LOW); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, -speed);
  } else {
    digitalWrite(IN1_pin, HIGH); digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, 0);
  }
}

void setMotorSpeedDifferential(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  int actualLeftSpeed = 0, actualRightSpeed = 0;
  int turnPWM = 0;

  if (rightSpeed > leftSpeed) {
    turnPWM = constrain(rightSpeed, MIN_PWM, MAX_PWM);
    actualLeftSpeed = -turnPWM; actualRightSpeed = turnPWM;
  } else if (leftSpeed > rightSpeed) {
    turnPWM = constrain(leftSpeed, MIN_PWM, MAX_PWM);
    actualLeftSpeed = turnPWM; actualRightSpeed = -turnPWM;
  }
  driveOneMotor(ENA, IN1, IN2, actualLeftSpeed);
  driveOneMotor(ENB, IN3, IN4, actualRightSpeed);
}

void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  driveOneMotor(ENA, IN1, IN2, speed);
  driveOneMotor(ENB, IN3, IN4, speed);
}

// =========================================================================
// ==                     센서 처리 및 각도 계산 함수                     ==
// =========================================================================

void getAccel_Data(void) {
  mpu.getAcceleration(&ax, &ay, &az);
  float accelSensitivity = 8192.0;
  Axyz[0] = (float)ax / accelSensitivity;
  Axyz[1] = (float)ay / accelSensitivity;
  Axyz[2] = (float)az / accelSensitivity;
}

void getGyro_Data(void) {
  mpu.getRotation(&gx, &gy, &gz);
  float gyroSensitivity = 65.5;
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
    mpu.setDLPFMode(MPU9250_DLPF_BW_42);
  } else {
    while (1); // 연결 실패 시 무한 루프
  }
}

// [!! 주의 !!] 이 함수는 이제 Serial(USB)로만 동작합니다.
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

void imuUpdate() {
  getAccel_Data();
  getCompass_Data_Raw();
  applyCompassCalibration();
  calculateTiltHeading();
}

// =========================================================================
// ==                       PID 제어 및 회전 함수                         ==
// =========================================================================

bool rotateToAbsAngle(float targetAbsAngle) {
  float currentAbsAngle = 0.0, error = 0.0;
  float integral = 0.0, lastError = 0.0, derivative = 0.0;
  int correction = 0;
  const float tolerance = 2.0; // 도
  unsigned long lastTime = micros(), startTime = millis();
  const unsigned long TIMEOUT_DURATION = 15000; // ms

  do {
    if (millis() - startTime > TIMEOUT_DURATION) {
      setMotorSpeedDifferential(0, 0); return false;
    }

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;
    if (deltaTime <= 0) deltaTime = 0.01;

    imuUpdate();
    currentAbsAngle = currentHeading;
    error = angleDiff(targetAbsAngle, currentAbsAngle);

    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = round(Kp * error + Ki * integral + Kd * derivative);

    int leftSpeed  = -correction;
    int rightSpeed = correction;
    setMotorSpeedDifferential(leftSpeed, rightSpeed);

    delay(10);

  } while (abs(error) > tolerance);

  setMotorSpeedDifferential(0, 0);
  return true;
}

// =========================================================================
// ==                       통신 및 명령어 처리 함수                      ==
// =========================================================================

bool parseCommand(const char* command, int &speed, float &angle, int &s_param) {
  const char* v_ptr = strchr(command, 'v');
  const char* a_ptr = strchr(command, 'a');
  const char* s_ptr = strchr(command, 's');

  if (v_ptr == NULL || a_ptr == NULL || s_ptr == NULL) {
    return false;
  }

  speed   = atoi(v_ptr + 1);
  angle   = atof(a_ptr + 1);
  s_param = atoi(s_ptr + 1);

  return true;
}


// [수정] Serial 포트로 완료 신호를 전송합니다.
void sendCompletionSignal() {
  imuUpdate();
  float finalRelativeAngle = angleDiff(currentHeading, initialHeading);

  Serial.print('a');
  Serial.print(finalRelativeAngle, 1); // 소수점 1자리
  Serial.println("s0"); // println이 \r\n을 붙여줌
}

// [수정] Serial 포트에서 입력을 처리합니다.
void handleUartInput() {
  
    //char data = mySerial.read(); // 1바이트 읽어서

  if (mySerial.available()) { // [수정] mySerial -> Serial
    char incomingByte = mySerial.read(); // [수정] mySerial -> Serial
    mySerial.write("Network confirm from A");
    
    Serial.write(incomingByte);
    if (incomingByte == '\n' || incomingByte == '\r') {
      if (uartBufferIndex > 0) {
        uartBuffer[uartBufferIndex] = '\0';

        int receivedSpeed;
        float receivedAngle;
        int receivedS;

        if (parseCommand(uartBuffer, receivedSpeed, receivedAngle, receivedS)) {
          targetSpeed = constrain(receivedSpeed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
          targetAngle = wrap360(initialHeading + receivedAngle);
          newCommandReceived = true;

          if (currentState == MOVING || currentState == ROTATING) {
            setMotorSpeedDifferential(0, 0);
            moveAtSpeed(0);
          }
          currentState = ROTATING;
        } else {
          // 명령어 형식 오류 (디버깅 없음)
        }
      }
      uartBufferIndex = 0;

    } else if (uartBufferIndex < (UART_BUFFER_SIZE - 1)) {
      uartBuffer[uartBufferIndex++] = incomingByte;
    }
  }
}


// =========================================================================
// ==                         메인 설정 및 루프                         ==
// =========================================================================

void setup() {
  Serial.begin(UART_BAUD_RATE); // [수정] 통신용 Serial 시작 (9600)
  mySerial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  imuSetup();
  //runCompassCalibration(); // 캘리브레이션은 B 연결 전에 USB(115200)로 해야 함

  moveAtSpeed(INITIAL_MOVE_SPEED); delay(INITIAL_MOVE_DURATION); moveAtSpeed(0);
  delay(1000);
  for (int i = 0; i < 10; i++) { imuUpdate(); delay(50); }
  initialHeading = currentHeading;
  targetAngle = initialHeading;

  currentState = IDLE;
}

void loop() {

  handleUartInput(); // Serial 로 입력 처리

  switch (currentState) {
    case IDLE:
      break;

    case ROTATING:
      if (newCommandReceived) {
        newCommandReceived = false;
        bool rotationSuccess = rotateToAbsAngle(targetAngle);
        if (rotationSuccess) {
          if (targetSpeed != 0) {
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(targetSpeed);
          } else {
            sendCompletionSignal(); // Serial 로 전송
            currentState = IDLE;
          }
        } else {
          sendCompletionSignal(); // Serial 로 전송
          currentState = IDLE;
        }
      }
      break;

    case MOVING:
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0);
        sendCompletionSignal(); // Serial 로 전송
        currentState = IDLE;
      }
      break;
  }

  delay(50);
}
