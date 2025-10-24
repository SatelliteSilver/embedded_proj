// header
#include <SoftwareSerial.h>

// Serial Define
SoftwareSerial BTSerial(2, 3);  //TX2/  RX3
// "FOR DEBUGGING" 디버깅용: 6,7번을 시리얼 통신용으로 사용합니다.
//SoftwareSerial mySerial(6, 7);

void setup() {
  // pinMode(8, OUTPUT);    // HC-05 핀 제어
  // digitalWrite(8, HIGH); // AT 사용 용도

  // for debugging
  // mySerial.begin(9600);

  Serial.begin(9600);   
  BTSerial.begin(9600); //At+UART:변경속도,0,0 (115200으로 변경시 사용)
  Serial.println("PC1 <-> PC2 bluetooth ready");  // ready
}

void loop() {
  if (BTSerial.available()) {
    int data = BTSerial.read();
    Serial.write(data);
  }

  if (Serial.available()) {
    int data = Serial.read();
    BTSerial.write(data);
  }
}

//아스키코드 그대로 출력
// #include <SoftwareSerial.h>

// SoftwareSerial BTSerial(2, 3);

// void setup() {
//   Serial.begin(115200);
//   BTSerial.begin(115200); 
// }

// void loop() {
//   if (BTSerial.available()) {
//     char c = BTSerial.read();
//     Serial.print((int)c);
//   }
  
//   if (Serial.available()) {
//     char c = Serial.read();
//     BTSerial.write(c);
//     Serial.print((int)c);
//   }
// }

