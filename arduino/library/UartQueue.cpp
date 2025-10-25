//
// UartQueue.cpp
// UartQueue 클래스의 멤버 함수 구현 (32bit 패킷 처리용)
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

// 큐에서 데이터를 꺼냅니다. (Dequeue)
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

// 32bit 패킷을 4바이트로 분할하여 큐에 추가
bool UartQueue::enqueue32bit(uint32_t packet)
{
    if (isFull() || (UART_QUEUE_SIZE - length) < 4) {
        return false; // 큐에 공간이 부족
    }
    
    // MSB부터 LSB까지 4바이트로 분할 (UART는 MSB/LSB 반전)
    uint8_t byte3 = (packet >> 24) & 0xFF;  // MSB
    uint8_t byte2 = (packet >> 16) & 0xFF;
    uint8_t byte1 = (packet >> 8) & 0xFF;
    uint8_t byte0 = packet & 0xFF;          // LSB
    
    // LSB부터 MSB 순서로 큐에 추가 (UART 반전 고려)
    if (!enqueue(byte0)) return false;
    if (!enqueue(byte1)) return false;
    if (!enqueue(byte2)) return false;
    if (!enqueue(byte3)) return false;
    
    return true;
}

// 큐에서 4바이트를 꺼내서 32bit 패킷으로 복원
bool UartQueue::dequeue32bit(uint32_t &packet)
{
    if (length < 4) {
        return false; // 4바이트 미만
    }
    
    uint8_t byte0, byte1, byte2, byte3;
    
    // LSB부터 MSB 순서로 큐에서 제거 (UART 반전 고려)
    if (!dequeue(byte0)) return false;
    if (!dequeue(byte1)) return false;
    if (!dequeue(byte2)) return false;
    if (!dequeue(byte3)) return false;
    
    // 32bit 패킷 복원
    packet = ((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) | 
             ((uint32_t)byte1 << 8) | byte0;
    
    return true;
}

// 32bit 패킷이 완성되었는지 확인 (최소 4바이트)
bool UartQueue::hasCompletePacket() const
{
    return (length >= 4);
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
