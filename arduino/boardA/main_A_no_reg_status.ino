#include <AltSoftSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"

AltSoftSerial imuSerial;
#define IMU_SERIAL imuSerial

UartQueue rxQueue;
UartQueue tempQueue;

struct SAngle { float Roll, Pitch, Yaw; };
struct SAcc   { float X, Y, Z; };
struct SGyro  { float X, Y, Z; };
SAngle stcAngle;
SAcc   stcAcc;
SGyro  stcGyro;

float Axyz[3] = {0}, Gxyz[3] = {0};
float currentHeading = 0.0f;
float initialHeading = 0.0f;
float targetAngle    = 0.0f;
int   targetSpeed    = 0;
bool  newCommandReceived = false;

float yawZeroOffset = 0.0f;
float lastOutYaw    = 0.0f;

enum RobotState { IDLE, ROTATING, MOVING };
RobotState currentState = IDLE;
unsigned long moveStartTime = 0;

float Kp = 4.0f;
float Ki = 0.0f;
float Kd = 0.1f;

#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 11
#define IN3 12
#define IN4 13

const int MIN_PWM = 70;
const int MAX_PWM = 100;
const int MAX_MOVE_PWM = 100;
const unsigned long MOVE_DURATION = 2000;

#define UART_BAUD_RATE 115200
#define DEBUG 1 // 이 값은 이제 의미가 없습니다.

static inline float wrap360(float angle) {
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

static inline float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

const float ANGLE_DEADBAND = 3.0f;
bool moving = false;
const float LPF_ALPHA = 0.25f;

void driveOneMotor(int EN_pin, int IN1_pin, int IN2_pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(IN1_pin, HIGH);
    digitalWrite(IN2_pin, LOW);
    analogWrite(EN_pin, speed);
  } else if (speed < 0) {
    digitalWrite(IN1_pin, LOW);
    digitalWrite(IN2_pin, HIGH);
    analogWrite(EN_pin, -speed);
  } else {
    digitalWrite(IN1_pin, LOW);
    digitalWrite(IN2_pin, LOW);
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
  if (abs(leftSpeed) < MIN_PWM && leftSpeed != 0) {
    leftSpeed = (leftSpeed > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(rightSpeed) < MIN_PWM && rightSpeed != 0) {
    rightSpeed = (rightSpeed > 0) ? MIN_PWM : -MIN_PWM;
  }
  driveOneMotor(ENA, IN1, IN2, leftSpeed);
  driveOneMotor(ENB, IN3, IN4, rightSpeed);
}

void moveAtSpeed(int speed) {
  speed = constrain(speed, -MAX_MOVE_PWM, MAX_MOVE_PWM);
  int corrected = -speed;
  moving = (abs(corrected) >= MIN_PWM);
  driveOneMotor(ENA, IN1, IN2, corrected);
  driveOneMotor(ENB, IN3, IN4, corrected);
}

const unsigned long WT901_BAUD = 9600;
uint8_t imuFrame[11];
uint8_t imuIdx = 0;
inline bool wt901CheckSumOK(const uint8_t* f) {
  uint8_t sum = 0;
  for (int i = 0; i < 10; ++i) sum += f[i];
  return (sum == f[10]);
}

void imuSetup() {
  IMU_SERIAL.begin(WT901_BAUD);
}

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
      if (imuFrame[1] == 0x51) {
        stcAcc.X = (float)s16(2, 3) / 32768.0f * 16.0f;
        stcAcc.Y = (float)s16(4, 5) / 32768.0f * 16.0f;
        stcAcc.Z = (float)s16(6, 7) / 32768.0f * 16.0f;
        Axyz[0] = stcAcc.X; Axyz[1] = stcAcc.Y; Axyz[2] = stcAcc.Z;
      } else if (imuFrame[1] == 0x52) {
        stcGyro.X = (float)s16(2, 3) / 32768.0f * 2000.0f;
        stcGyro.Y = (float)s16(4, 5) / 32768.0f * 2000.0f;
        stcGyro.Z = (float)s16(6, 7) / 32768.0f * 2000.0f;
        Gxyz[0] = stcGyro.X; Gxyz[1] = stcGyro.Y; Gxyz[2] = stcGyro.Z;
      } else if (imuFrame[1] == 0x53) {
        stcAngle.Roll  = (float)s16(2, 3) / 32768.0f * 180.0f;
        stcAngle.Pitch = (float)s16(4, 5) / 32768.0f * 180.0f;
        stcAngle.Yaw   = (float)s16(6, 7) / 32768.0f * 180.0f;
      }
    }
  }
}

void imuUpdate() {
  imuPollWT901();
}

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

bool rotateToAbsAngle(float targetAbsAngle) {
  float error = 0.0f;
  float integral = 0.0f, lastError = 0.0f, derivative = 0.0f;
  int correction = 0;
  unsigned long lastTime = micros(), startTime = millis();
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
    if (abs(error) <= ANGLE_DEADBAND) {
      setMotorSpeedDifferential(0, 0);
      return true;
    }
    integral += error * deltaTime;
    derivative = (deltaTime > 0) ? (error - lastError) / deltaTime : 0;
    lastError = error;
    correction = (int)round(Kp * error + Ki * integral + Kd * derivative);
    if (abs(correction) < MIN_PWM) {
      correction = (correction >= 0) ? MIN_PWM : -MIN_PWM;
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
    setMotorSpeedDifferential(leftSpeed, rightSpeed);
    delay(10);
  }
}

void sendStatusPacket() {
  uint32_t status_packet = (1UL << 9);
  uint8_t b1 = (status_packet >> 24) & 0xFF;
  uint8_t b2 = (status_packet >> 16) & 0xFF;
  uint8_t b3 = (status_packet >> 8)  & 0xFF;
  uint8_t b4 = status_packet & 0xFF;
  Serial.write(b1);
  Serial.write(b2);
  Serial.write(b3);
  Serial.write(b4);
}

void setup() {
  Serial.begin(UART_BAUD_RATE);
  imuSetup();
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  unsigned long t0 = millis();
  uint16_t angSamples = 0;
  float lastYawSample = 9999.0f;
  while (millis() - t0 < 2000) {
    imuUpdate();
    updateHeadingFused();
    if (stcAngle.Yaw != lastYawSample) {
      lastYawSample = stcAngle.Yaw;
      angSamples++;
      if (angSamples >= 5) break;
    }
    delay(10);
  }
  yawZeroOffset = lastOutYaw;
  updateHeadingFused();
  initialHeading = currentHeading;
  targetAngle    = initialHeading;
  setMotorSpeedDifferential(0, 0);
  moveAtSpeed(0);
  currentState = IDLE;
  newCommandReceived = false;
  targetSpeed = 0;
}

uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void loop() {
  while (Serial.available()) {
    uint8_t b = reverseBits(Serial.read());
    rxQueue.enqueue(b);
  }

  while (rxQueue.getLength() >= 4) {
    uint8_t b1, b2, b3, b4;
    rxQueue.dequeue(b1);
    rxQueue.dequeue(b2);
    rxQueue.dequeue(b3);
    rxQueue.dequeue(b4);

    uint8_t reversed_b1 = reverseBits(b1);
    uint8_t reversed_b2 = reverseBits(b2);
    uint8_t reversed_b3 = reverseBits(b3);
    uint8_t reversed_b4 = reverseBits(b4);

    uint32_t packet = ((uint32_t)reversed_b3 << 24) |
                      ((uint32_t)reversed_b4 << 16) |
                      ((uint32_t)reversed_b1 << 8)  |
                      reversed_b2;

    uint8_t speed_type = (packet >> 30) & 0x03;
    uint8_t speed_val  = (packet >> 22) & 0xFF;
    uint8_t angle_type = (packet >> 20) & 0x03;
    uint8_t angle_val  = (packet >> 12) & 0xFF;
    uint8_t mode_type  = (packet >> 10) & 0x03;
    uint8_t mode_val   = (packet >> 2)  & 0xFF;

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
        continue;
    }

    int new_v = 0;
    float new_a = 0.0f;

    if (speed_type == 0x01) {
        new_v = speed_val;
    }
    if (angle_type == 0x02) {
        new_a = (float)((int8_t)angle_val);
    }

    if (new_v > 0 || new_a != 0.0f) {
        targetSpeed = constrain(new_v, 0, MAX_MOVE_PWM);
        targetAngle = wrap360(currentHeading + new_a);
        setMotorSpeedDifferential(0, 0);
        moveAtSpeed(0);
        imuUpdate();
        updateHeadingFused();
        float err = angleDiff(targetAngle, currentHeading);
        if (abs(err) <= ANGLE_DEADBAND) {
            if (targetSpeed > 0) {
                currentState = MOVING;
                moveStartTime = millis();
                moveAtSpeed(max(targetSpeed, MIN_PWM));
            } else {
                sendStatusPacket();
                currentState = IDLE;
            }
        } else {
            newCommandReceived = true;
            currentState = ROTATING;
        }
    }
  }

  switch (currentState) {
    case IDLE:
      break;
    case ROTATING:
      if (newCommandReceived) {
        newCommandReceived = false;
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
          break;
        }
        if (rotateToAbsAngle(targetAngle)) {
          if (targetSpeed != 0) {
            currentState = MOVING;
            moveStartTime = millis();
            moveAtSpeed(max(targetSpeed, MIN_PWM));
          } else {
            sendStatusPacket();
            currentState = IDLE;
          }
        } else {
          currentState = IDLE;
        }
      }
      break;

    case MOVING:
      if (millis() - moveStartTime >= MOVE_DURATION) {
        moveAtSpeed(0);
        sendStatusPacket();
        currentState = IDLE;
      }
      break;
  }
  imuUpdate();
  updateHeadingFused();
  delay(20);
}
