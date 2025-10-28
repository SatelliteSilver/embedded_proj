#include <SoftwareSerial.h>
#include "UartQueue.h"
#include "PacketProtocol.h"
#include <Arduino.h> // Standard Arduino functions

// SoftwareSerial (PC <-> B)'

SoftwareSerial btSerial(2, 3);  // RX=2, TX=3

UartQueue rxQueue;      // Used for PC -> A command buffering
UartQueue statusRxQueue; // New: Used for A -> PC status buffering

void setup() {
    // 1. Hardware UART (D0/D1) communication with Arduino A (9600 bps)
    Serial.begin(9600); 

    // 2. SoftwareSerial (D2/D3) communication with PC via BT Module (9600 bps)
    btSerial.begin(9600); 
}

void loop() {
    // 1. PC (BT) -> B 수신 (명령 바이트를 큐에 적재)
    while (btSerial.available()) {
        uint8_t b = btSerial.read();
        rxQueue.enqueue(b);
    }
    
    // --- NEW: A (UART) -> B 수신 (상태 바이트를 큐에 적재) ---
    while (Serial.available()) {
        uint8_t b = Serial.read();
        statusRxQueue.enqueue(b);
    }

    // 2. PC -> B -> A 명령 처리 (4바이트 큐에서 꺼내 A로 전송)
    while (rxQueue.getLength() >= 4) {
        uint8_t b1, b2, b3, b4;
        // Dequeue data in the order it arrived
        rxQueue.dequeue(b1);
        rxQueue.dequeue(b2);
        rxQueue.dequeue(b3);
        rxQueue.dequeue(b4);
        
        // Send directly to Arduino A via Hardware UART (D0/D1)
        Serial.write(b1);
        Serial.write(b2);
        Serial.write(b3);
        Serial.write(b4);
    }

    // 3. A -> B -> PC 상태 중계 (4바이트 큐에서 꺼내 PC로 전송)
    while (statusRxQueue.getLength() >= 4) {
        uint8_t b1, b2, b3, b4;
        statusRxQueue.dequeue(b1);
        statusRxQueue.dequeue(b2);
        statusRxQueue.dequeue(b3);
        statusRxQueue.dequeue(b4);
        
        // Send the 4-byte packet to PC via Bluetooth
        btSerial.write(b1);
        btSerial.write(b2);
        btSerial.write(b3);
        btSerial.write(b4);
    }
}
