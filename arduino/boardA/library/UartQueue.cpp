//
// UartQueue.cpp
// UartQueue 클래스의 멤버 함수 구현
//

#include "UartQueue.h"

// 생성자: clear 함수를 호출하여 멤버 변수들을 초기화합니다.
UartQueue::UartQueue()
{
    clear();
}

// 큐를 비우고 초기 상태로 설정합니다.
void UartQueue::clear()
{
    front = UART_QUEUE_SIZE - 1;
    rear = UART_QUEUE_SIZE - 1;
    length = 0;
    // data 배열의 내용을 굳이 지울 필요는 없습니다.
    // front/rear/length로 접근을 제어하기 때문입니다.
}

// 큐가 가득 찼는지 확인합니다.
bool UartQueue::isFull() const 
{
    return (length == UART_QUEUE_SIZE);
}

// 큐가 비었는지 확인합니다.
bool UartQueue::isEmpty() const 
{
    return (length == 0);
}

// 현재 큐의 길이를 반환합니다.
int UartQueue::getLength() const
{
    return length;
}

// 큐의 뒤쪽에 새 항목을 추가합니다.
bool UartQueue::enqueue(uint8_t newItem)
{
    if (isFull()) {
        // Serial.println("[ERROR] Queue is Full. Enqueue Failed."); // 필요시 디버깅
        return false; // 큐가 가득 차서 실패
    }
    
    // rear 인덱스를 (순환) 증가시킵니다.
    rear = (rear + 1) % UART_QUEUE_SIZE;
    data[rear] = newItem;
    length++;
    
    return true; // 성공
}

// 큐의 앞쪽에서 항목을 제거하고 반환합니다.
bool UartQueue::dequeue(uint8_t &item) 
{
    if (isEmpty()) {
        // Serial.println("[ERROR] Queue is Empty. Dequeue Failed."); // 필요시 디버깅
        return false; // 큐가 비어서 실패
    }
    
    // front 인덱스를 (순환) 증가시킵니다.
    front = (front + 1) % UART_QUEUE_SIZE;
    item = data[front]; // 참조 변수를 통해 데이터 반환
    length--;
    
    return true; // 성공
}


// (디버깅용) 큐의 내용을 시리얼 모니터에 출력합니다.
void UartQueue::printQueue() const
{
    if (isEmpty()) {
        Serial.println("[EMPTY QUEUE]");
        return;
    }
    
    Serial.print("Queue (front->rear): [ ");
    
    int i = front;
    
    // length 기반으로 순회하는 것이 더 안전합니다.
    for (int c = 0; c < length; c++) {
        i = (i + 1) % UART_QUEUE_SIZE;
        Serial.print(data[i]);
        Serial.print(" ");
    }
    
    Serial.println("]");
}