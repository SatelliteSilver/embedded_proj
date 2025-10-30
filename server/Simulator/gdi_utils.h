#pragma once
#include <windows.h>
#include <stdio.h>
#include <algorithm>

/**
 * @brief 지정된 HDC에 사각형을 그립니다.
 * @param hdc 그리기 대상 Device Context
 * @param left 사각형의 왼쪽 x 좌표
 * @param top 사각형의 위쪽 y 좌표
 * @param right 사각형의 오른쪽 x 좌표
 * @param bottom 사각형의 아래쪽 y 좌표
 * @note 현재 HDC에 선택된 펜(Pen)과 브러시(Brush)를 사용하여 그립니다.
 */
void DrawRect(HDC hdc, RECT rect);
void DrawEllipse(HDC hdc, RECT rect);
RECT MakeRect(int left, int top, int right, int bottom);
RECT MakeRectCenter(int xCenter, int yCenter, int xlength, int ylength);
RECT MakeEllipseCenter(int xCenter, int yCenter, int radius);
//void DrawRectCenter(HDC hdc, int xCenter, int yCenter, int xlength, int ylength);

/**
 * @brief 지정된 HDC에 원을 그립니다.
 * @param hdc 그리기 대상 Device Context
 * @param x 원의 중심 x 좌표
 * @param y 원의 중심 y 좌표
 * @param radius 원의 반지름
 * @note 현재 HDC에 선택된 펜(Pen)과 브러시(Brush)를 사용하여 그립니다.
 */
void DrawCircle(HDC hdc, int x, int y, int radius);

/**
 * @brief 지정된 HDC에 직선을 그립니다.
 * @param hdc 그리기 대상 Device Context
 * @param startX 선의 시작 x 좌표
 * @param startY 선의 시작 y 좌표
 * @param endX 선의 끝 x 좌표
 * @param endY 선의 끝 y 좌표
 * @note 현재 HDC에 선택된 펜(Pen)을 사용하여 그립니다.
 */

void DrawLine(HDC hdc, int startX, int startY, int endX, int endY);

/**
 * @brief 지정된 HDC에 텍스트를 출력합니다.
 * @param hdc 그리기 대상 Device Context
 * @param x 텍스트가 시작될 x 좌표
 * @param y 텍스트가 시작될 y 좌표
 * @param text 출력할 유니코드 텍스트 (L"..." 형태)
 * @note 현재 HDC에 설정된 텍스트 색상(SetTextColor)과 배경 모드(SetBkMode)를 사용합니다.
 */
void DrawText(HDC hdc, int x, int y, const wchar_t* text);

/////// TEXT

void DrawDebugText(HDC hdc, int x, int y, const WCHAR* format, ...);

bool checkRectCircleCollision(const RECT& rect, float circleX, float circleY, float circleRadius);

