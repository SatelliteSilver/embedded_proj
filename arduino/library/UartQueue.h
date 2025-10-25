//
// UartQueue.h
// 아두이노 UART 통신을 위한 C++ 기반 원형 큐 (32bit 패킷 처리용)
//

#ifndef UART_QUEUE_H
#define UART_QUEUE_H

#include <Arduino.h> // 아두이노 기본 헤더 포함

// 큐의 최대 크기를 정의합니다. 32bit = 4byte이므로 더 큰 큐가 필요합니다.
#define UART_QUEUE_SIZE 256 

class UartQueue
{
private:
    uint8_t data[UART_QUEUE_SIZE]; // 데이터를 저장할 고정 크기 배열 (byte)
    int     front;      // 큐의 시작 인덱스
    int     rear;       // 큐의 끝 인덱스
    int     length;     // 현재 큐에 저장된 항목 수

public:
    // 생성자: 큐를 초기화합니다.
    UartQueue();
    
    // 큐에 데이터를 추가합니다. (Enqueue)
    // 성공 시 true, 큐가 가득 찼으면 false 반환
    bool enqueue(uint8_t newItem);
    
    // 큐에서 데이터를 꺼냅니다. (Dequeue)
    // 성공 시 true, 큐가 비었으면 false 반환
    // 꺼낸 데이터는 'item' 참조 변수에 저장됩니다.
    bool dequeue(uint8_t &item); 
    
    // 32bit 패킷을 4바이트로 분할하여 큐에 추가
    bool enqueue32bit(uint32_t packet);
    
    // 큐에서 4바이트를 꺼내서 32bit 패킷으로 복원
    bool dequeue32bit(uint32_t &packet);
    
    // 큐를 비웁니다.
    void clear();
    
    // 큐가 가득 찼는지 확인합니다.
    bool isFull() const;
    
    // 큐가 비었는지 확인합니다.
    bool isEmpty() const;
    
    // 현재 큐에 있는 항목의 수를 반환합니다.
    int getLength() const;
    
    // 32bit 패킷이 완성되었는지 확인 (최소 4바이트)
    bool hasCompletePacket() const;
    
    // (디버깅용) 큐의 현재 상태를 시리얼 모니터에 출력합니다.
    void printQueue() const;
};

#endif // UART_QUEUE_H
