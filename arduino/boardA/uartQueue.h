//
// UartQueue.h
// 아두이노 UART 통신을 위한 C++ 기반 원형 큐 (고정 크기)
//

#ifndef UART_QUEUE_H
#define UART_QUEUE_H

#include <Arduino.h> // 아두이노 기본 헤더 포함

// 큐의 최대 크기를 정의합니다. 이 값은 필요에 따라 조절할 수 있습니다.
#define UART_QUEUE_SIZE 128 

class UartQueue
{
private:
    uint8_t data[UART_QUEUE_SIZE]; // 데이터를 저장할 고정 크기 배열 (byte)
    int     front;      // 큐의 시작 인덱스
    int     rear;       // 큐의 끝 인덱스
    int     length;     // 현재 큐에 저장된 항목 수
    // maxQueue는 이제 UART_QUEUE_SIZE 상수로 대체됩니다.

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
    
    // 큐를 비웁니다.
    void clear();
    
    // 큐가 가득 찼는지 확인합니다.
    bool isFull() const;
    
    // 큐가 비었는지 확인합니다.
    bool isEmpty() const;
    
    // 현재 큐에 있는 항목의 수를 반환합니다.
    int getLength() const;
    
    // (디버깅용) 큐의 현재 상태를 시리얼 모니터에 출력합니다.
    void printQueue() const;
};

#endif // UART_QUEUE_H