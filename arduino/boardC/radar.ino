#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// ---------------- 핀 정의 ----------------
#define TRIG_PIN PD6
#define ECHO_PIN PD7
#define RXD_PIN  PD0   // UART RXD (PCINT16)

// ---------------- 전역 변수 ----------------
volatile unsigned long duration;
volatile int distance;

volatile int direction = 1;        // 1=15→165, -1=165→15
volatile uint8_t waiting = 0;
volatile uint8_t trigger_flag = 0;

// ---------------- UART (송신만) ----------------
void UART_init(void) {
    uint16_t ubrr = 8; // 115200bps @16MHz
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void UART_tx(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_print(const char *str) {
    while (*str) UART_tx(*str++);
}

void UART_printInt(int num) {
    char buf[10];
    itoa(num, buf, 10);
    UART_print(buf);
}

// ---------------- Servo (Timer1, OC1A=D9) ----------------
void Servo_init(void) {
    DDRB |= (1 << PB1);
    // Fast PWM mode 14, prescale=8
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999; // 50Hz (20ms)
}

void Servo_write(int angle) {
    unsigned int pulse = 600 + ((long)angle * 1900 / 180); // 600~2500us
    OCR1A = pulse * 2; // prescale8→0.5us tick
}

// ---------------- 초음파 거리 측정 (160cm 한정) ----------------
int calculateDistance(void) {
    // 트리거 펄스
    PORTD &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTD |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIG_PIN);
    _delay_us(200); // 모듈 안정화

    // Echo 핀 HIGH 대기 (최대 160cm 왕복시간 ≈ 9.5ms)
    uint16_t timeout = 9500;
    while (!(PIND & (1 << ECHO_PIN)) && timeout--) _delay_us(1);
    if (!timeout) return 0; // Echo 안 들어옴

    // Echo HIGH 유지 시간 측정 (최대 9.5ms)
    TCNT1 = 0;
    timeout = 9500;
    while ((PIND & (1 << ECHO_PIN)) && timeout--) _delay_us(1);
    duration = TCNT1;

    // 거리 계산: cm = (duration * 0.00857f)
    int dist = (int)(duration * 0.00857f);
    if (dist > 160) dist = 0; // 160cm 초과는 무효
    return dist;
}

// ---------------- RXD 스타트비트 감지용 PCINT ----------------
ISR(PCINT2_vect) {
    static uint8_t prev = (1 << RXD_PIN);
    uint8_t now = PIND & (1 << RXD_PIN);
    if (prev && !now) { // 하강에지
        if (waiting) trigger_flag = 1;
    }
    prev = now;
}

void RXStartBitInterrupt_init(void) {
    DDRD &= ~(1 << RXD_PIN);
    PCICR  |= (1 << PCIE2);    // PortD 그룹
    PCMSK2 |= (1 << PCINT16);  // PD0 마스크
}

// ---------------- 메인 루프 ----------------
int main(void) {
    DDRD |= (1 << TRIG_PIN);
    DDRD &= ~(1 << ECHO_PIN);
    PORTD &= ~(1 << TRIG_PIN);

    UART_init();
    Servo_init();
    RXStartBitInterrupt_init();
    sei();

    UART_print("System Ready - Waiting for RX start bit...\n");

    while (1) {
        // ---- 방향별 스캔 ----
        if (direction == 1) {
            for (int i = 15; i <= 165; i++) {
                Servo_write(i);
                _delay_ms(60);               // 서보 안정화
                int dist = calculateDistance();

                UART_printInt(i); UART_tx(',');
                UART_printInt(dist); UART_tx('\n');
                _delay_ms(30);               // 틱 간격 총합 ≈90ms
            }
        } else {
            for (int i = 165; i >= 15; i--) {
                Servo_write(i);
                _delay_ms(60);
                int dist = calculateDistance();

                UART_printInt(i); UART_tx(',');
                UART_printInt(dist); UART_tx('\n');
                _delay_ms(30);
            }
        }

        // ---- 스캔 완료 후 대기 ----
        waiting = 1;
        trigger_flag = 0;
        UART_print(direction == 1 ?
                   "HOLD @165. Waiting RX start bit...\n" :
                   "HOLD @15.  Waiting RX start bit...\n");

        while (!trigger_flag) _delay_ms(1);

        trigger_flag = 0;
        waiting = 0;
        direction *= -1;
    }
}
