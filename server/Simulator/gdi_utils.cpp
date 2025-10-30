#define NOMINMAX

#include "gdi_utils.h"


void DrawRect(HDC hdc, RECT rect)
{
    // Win32 API의 Rectangle 함수 사용
    // 현재 선택된 펜(Pen)으로 외곽선을 그리고,
    // 현재 선택된 브러시(Brush)로 내부를 채웁니다.
    Rectangle(hdc, rect.left, rect.top, rect.right, rect.bottom);
}

void DrawEllipse(HDC hdc, RECT rect)
{
    Ellipse(hdc, rect.left, rect.top, rect.right, rect.bottom);
}

RECT MakeRectCenter(int xCenter, int yCenter, int xlength, int ylength)
{
    RECT rect;
    rect.left = xCenter - xlength;
    rect.right = xCenter + xlength;
    rect.top = yCenter - ylength;
    rect.bottom = yCenter + ylength;
    
    return rect;
}

RECT MakeRect(int left, int top, int right, int bottom)
{
    RECT rect;
    rect.left = left;
    rect.right = right;
    rect.top = top;
    rect.bottom = bottom;

    return rect;
}

RECT MakeEllipseCenter(int xCenter, int yCenter, int radius)
{
    RECT rect;
    rect.left = xCenter - radius;
    rect.right = xCenter + radius;
    rect.top = yCenter - radius;
    rect.bottom = yCenter + radius;

    return rect;
}

void DrawCircle(HDC hdc, int x, int y, int radius)
{
    // Win32 API의 Ellipse 함수 사용
    // 원은 중심점 (x, y)와 반지름(radius)을 기준으로
    // 사각형 영역(left, top, right, bottom)을 계산하여 그립니다.
    int left = x - radius;
    int top = y - radius;
    int right = x + radius;
    int bottom = y + radius;
    Ellipse(hdc, left, top, right, bottom);
}

void DrawLine(HDC hdc, int startX, int startY, int endX, int endY)
{
    // Win32 API의 MoveToEx/LineTo 함수 사용
    // 1. 현재 펜의 위치(CP)를 시작점으로 이동
    MoveToEx(hdc, startX, startY, NULL);
    // 2. 현재 위치(시작점)에서 끝점까지 선을 그림
    LineTo(hdc, endX, endY);
}

void DrawText(HDC hdc, int x, int y, const wchar_t* text)
{
    if (text)
    {
        // Win32 API의 TextOutW (유니코드 버전) 함수 사용
        // lstrlenW로 문자열의 길이를 계산합니다.
        TextOutW(hdc, x, y, text, lstrlenW(text));
    }
}

void DrawDebugText(HDC hdc, int x, int y, const WCHAR* format, ...)
{
    // 1. 텍스트를 포맷팅하여 저장할 버퍼(메모리 공간) 선언
    WCHAR buffer[256];

    // 2. "..."으로 들어온 가변 인자들을 처리
    va_list args;           // 가변 인자 목록 포인터
    va_start(args, format); // 'format' 매개변수 뒤부터 가변 인자 시작

    // vswprintf_s: 가변 인자(va_list)를 받아 포맷에 맞게 버퍼에 씀
    vswprintf_s(buffer, _countof(buffer), format, args);

    va_end(args);           // 가변 인자 처리 끝

    // 3. 텍스트 그리기 속성 설정
    SetTextColor(hdc, RGB(0, 0, 0)); // 텍스트 색상 (검은색)
    SetBkMode(hdc, TRANSPARENT);       // 텍스트 배경 투명

    // 4. 텍스트를 그릴 영역 RECT 정의 (x, y를 시작점으로)
    RECT textRect;
    textRect.left = x;
    textRect.top = y;
    textRect.right = x + 400; // (그릴 영역을 400px 너비로 넉넉하게 잡음)
    textRect.bottom = y + 200;// (그릴 영역을 200px 높이로 넉넉하게 잡음)

    // 5. 화면(hdc)에 텍스트 그리기
    DrawTextW(
        hdc,
        buffer,       // 2번에서 완성된 문자열
        -1,           // 문자열 길이를 자동으로 계산
        &textRect,    // 4번에서 정의한 RECT 영역
        DT_LEFT | DT_TOP // 왼쪽 위 정렬 (이 플래그가 \n 줄바꿈도 처리해 줌)
    );
}

bool checkRectCircleCollision(const RECT& rect, float circleX, float circleY, float circleRadius) {

    // 1. 원의 중심에서 가장 가까운 사각형 위의 점(closestX, closestY)을 찾습니다.
    //    RECT의 long 좌표를 float으로 형 변환(casting)하여 비교합니다.
    float closestX = std::max((float)rect.left, std::min(circleX, (float)rect.right));
    float closestY = std::max((float)rect.top, std::min(circleY, (float)rect.bottom));

    // 2. 가장 가까운 점과 원의 중심 사이의 거리(의 제곱)를 계산합니다.
    //    모든 변수가 float이므로 델타 값도 float입니다.
    float deltaX = circleX - closestX;
    float deltaY = circleY - closestY;

    // 3. 거리의 제곱 (long long 대신 float 사용)
    float distSquared = (deltaX * deltaX) + (deltaY * deltaY);
    float radiusSquared = circleRadius * circleRadius;

    // 4. 거리의 제곱이 반지름의 제곱보다 작거나 같으면 충돌입니다.
    return distSquared <= radiusSquared;
}
