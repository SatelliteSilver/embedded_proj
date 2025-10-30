// car_simulation.cpp : 애플리케이션에 대한 진입점을 정의합니다.
//

#define NOMINMAX

#include "framework.h"
#include "car_simulation.h"
#include <cmath>
#define MAX_LOADSTRING 100
#define WINSIZEX 800
#define WINSIZEY 600
#define TIMER_ID 1
#define TIMERATE 8 // 8: 120FPS, 17: 60FPS, 33: 30FPS
#define PIE 3.141592
#define carSizeX 28
#define carSizeY 44
#define obstacleSize 16.5
#define range 25
struct Point2D {
    double x, y;
};

// 전역 변수:
HINSTANCE hInst;                                // 현재 인스턴스입니다.
WCHAR szTitle[MAX_LOADSTRING];                  // 제목 표시줄 텍스트입니다.
WCHAR szWindowClass[MAX_LOADSTRING];            // 기본 창 클래스 이름입니다.


// 이 코드 모듈에 포함된 함수의 선언을 전달합니다:
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);
    
    // TODO: 여기에 코드를 입력합니다.

    // 전역 문자열 초기화
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_CARSIMULATION, szWindowClass, MAX_LOADSTRING);
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CARSIMULATION));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_CARSIMULATION);
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    if (!RegisterClassExW(&wcex))
    {
        MessageBox(NULL, L"윈도우 클래스 등록 실패!", L"오류", MB_OK);
        return FALSE;
    }

    RECT rect = { 0, 0, WINSIZEX, WINSIZEY };
    DWORD dwStyle = WS_OVERLAPPEDWINDOW;

    AdjustWindowRect(&rect, dwStyle, TRUE);

    int windowWidth = rect.right - rect.left;
    int windowHeight = rect.bottom - rect.top;

    // 윈도우 생성
    HWND hWnd = CreateWindowW(
        szWindowClass,      // 1. 등록된 클래스 이름
        szTitle,            // 2. 창 제목
        dwStyle,            // 3. 창 스타일
        CW_USEDEFAULT,      // 4. x 위치 (Windows가 결정)
        CW_USEDEFAULT,      // 5. y 위치 (Windows가 결정)
        windowWidth,        // 6. ★ 계산된 "전체" 가로 크기
        windowHeight,       // 7. ★ 계산된 "전체" 세로 크기
        nullptr,            // 8. 부모 윈도우
        nullptr,            // 9. 메뉴 핸들 (클래스에서 사용)
        hInstance,          // 10. 인스턴스 핸들
        nullptr             // 11. 추가 파라미터
    );

    if (!hWnd)
    {
        MessageBox(NULL, L"윈도우 생성 실패!", L"오류", MB_OK);
        return FALSE;
    }

    // 윈도우 화면 표시
    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CARSIMULATION));

    MSG msg;

    // 기본 메시지 루프입니다:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}


BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 인스턴스 핸들을 전역 변수에 저장합니다.

   HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    // random Seed //
    srand((unsigned)time(NULL));
    
    // variables //
    static int phase, sign;
    static int destX1, destY1, destX2, destY2;
    static int CollisionCount, position;
    static float predictX1, predictY1, predictX2, predictY2;
    static float angle, ang, temp;
    static float AngleError;
    static float carX, carY, obstacleX, obstacleY;
    static RECT car;
    static RECT obstacle;
    static float FInalAngle;
    static bool g_bIsGameOver = false;
    static POINT carCorners[4];
    ofstream outFile("car_data.txt");
    
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // 메뉴 선택을 구문 분석합니다:
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_CREATE:
    {
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////

                                    // init main //
        //system init
        SetTimer(hWnd, TIMER_ID, TIMERATE, NULL);
        

        //variable init
        phase = sign = position;
        obstacleX = obstacleY = 0;
        carX = WINSIZEX / 2;           carY = WINSIZEY / 2;
        
        
        destX1 = destY1 = destX2 = destY2 = 0;
        angle = 45; ang = 0; CollisionCount = 0;
        
        
        FInalAngle = angle * PIE / 180 + AngleError * PIE / 180;
        // angleerror 는 도 단위임. 따라서 사인 함수에는 라디안으로 넣어야함
        car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
        obstacle= MakeEllipseCenter(obstacleX, obstacleY, 25);
        

        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////
        
        break;
    }
    case WM_TIMER:
    {
        // 여러 타이머가 있을 수 있으므로, 우리가 설정한 ID(1번)가 맞는지 확인
        if (wParam == TIMER_ID)
        {
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////

            if (g_bIsGameOver) {
                       ang = 0;
                       carX = WINSIZEX / 2; carY = WINSIZEY / 2;
                       g_bIsGameOver = false;
                       phase = 0;                       
            }
                                    // update main //

            getRotatedRectCorners(car, ang, carCorners);
            if (checkPolygonCircleCollision(carCorners, obstacleX, obstacleY, obstacleSize)) 
            {
                // 충돌 발생!
                g_bIsGameOver = true;
                CollisionCount++;
                //MessageBox(hWnd, TEXT("충돌!!"), TEXT("게임 오버"), MB_OK | MB_ICONSTOP);
                //PostQuitMessage(0);
            }
            

            switch (phase)
            {
                //장애물 탐지 및 점찍기
            case 0:

                position = 0;//rand() % 2;
                AngleError = rand() % 7 - 4;
                predictX1 = rand() % range;
                predictY1 = rand() % range;
                predictX2 = rand() % range;
                predictY2 = rand() % range;

                if (position == 0) // 왼쪽 obstacle
                {
                    obstacleX = WINSIZEX / 2 - 150; obstacleY = WINSIZEY / 2 - 100;
                        destX1 = obstacleX + obstacleSize + predictX1 * 5;
                        destY1 = obstacleY - obstacleSize - predictY1 * 5;
                        destX2 = obstacleX - obstacleSize - predictX2 * 10;
                        destY2 = obstacleY - obstacleSize - predictY2 * 3;
                }
                else //오른쪽 obstacle
                {
                    obstacleX = WINSIZEX / 2 + 150; obstacleY = WINSIZEY / 2 - 100;
                        destX1 = obstacleX - obstacleSize - predictX1 * 5;
                        destY1 = obstacleY - obstacleSize - predictY1 * 5;
                        destX2 = obstacleX + obstacleSize + predictX2 * 10;
                        destY2 = obstacleY - obstacleSize - predictY2 * 3;
                }
                obstacle = MakeEllipseCenter(obstacleX, obstacleY, obstacleSize);
                // 장애물 탐지 
                // 점찍기
                car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
                getRotatedRectCorners(car, ang, carCorners);
                // predict 랜덤값 주기 0 ~ 100까지
                // 0 에서 가까운 값일수록 자르기, 예를들면 5가 충돌이면 1은 당연히 충돌일 것임
                // predict 절대값이 작을수록 보상 크고 충돌은 무조건 안해야함.
                // 
               
                
                phase = 1;
                    
                break;

            case 1:
                car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
                obstacle = MakeEllipseCenter(obstacleX, obstacleY, obstacleSize);
                
                // dest x y 로 몸체 돌리기
                // 각도 계산
                angle = acos((carX - destX1) / sqrt((carX - destX1) * (carX - destX1) + (carY - destY1) * (carY - destY1)));
                if (angle < 0) angle *= -1;
                FInalAngle = angle + AngleError * PIE / 180;
                angle += FInalAngle;
                

                if (position == 0) // 왼쪽 obstacle
                {
                    if (carX > destX1)
                    {
                        ang -= 1;
                        // 좌상향
                        if (abs(ang) >= getAngleBetween(carX, carY, carX, carY - 10, destX1, destY1))
                        {
                            phase = 2;
                            temp = getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2);
                        }
                    }
                    else 
                    {
                        ang += 1;
                        if (abs(ang) <= getAngleBetween(carX, carY, carX, carY - 10, destX1, destY1))
                        {
                            phase = 2;
                            temp = getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2);
                        }
                    }
                }
                else // 오른쪽 obstacle
                {
                    if (carX < destX1)
                    {
                        ang += 1;
                        // 우상향
                        if (ang >= abs(getAngleBetween(carX, carY, carX, carY - 10, destX1, destY1)))
                        {
                            phase = 2;
                            temp = getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2);
                        }
                    }
                    else // 좌상향
                    {
                        ang -= 1;
                        if (abs(ang) >= abs(getAngleBetween(carX, carY, carX, carY - 10, destX1, destY1)))
                        {
                            phase = 2;
                            temp = getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2);
                        }
                    }
                }

                // 몸돌리기는 Render 파트에서

                break;

            case 2:
                car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
                obstacle = MakeEllipseCenter(obstacleX, obstacleY, obstacleSize);

                if (position == 0)
                {
                    if (carX < destX1 && phase == 2)
                    {
                        carX += 2 * cos(FInalAngle);
                    }
                    if (carX > destX1 && phase == 2) // 장애물이 왼쪽일 때
                    {
                        carX -= 2 * cos(FInalAngle);
                    }
                    if (carY > destY1)
                    {
                        carY -= 2 *sin(FInalAngle);
                        if(carY <= destY1)
                        {
                            phase = 3;
                        
                            break;
                        }
                    }
                }
                else // 우측 obstacle
                {
                    if (carX < destX1 && phase == 2)
                    {
                        carX -= 2 * cos(FInalAngle);
                    }
                    if (carX > destX1 && phase == 2) // 장애물이 왼쪽일 때
                    {
                        carX += 2 * cos(FInalAngle);
                    }
                    if (carY > destY1)
                    {
                        carY -= 2 * sin(FInalAngle);
                        if (carY <= destY1)
                        {
                            phase = 3;

                            break;
                        }
                    }
                }
                
                break;

            case 3: //돌 때
                obstacle = MakeEllipseCenter(obstacleX, obstacleY, obstacleSize);

                if (position == 0) // 좌측
                {
                    // destXY2 우상향
                    if (abs(ang) > temp) {
                        ang += 1;
                        if (abs(ang) <= getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2))
                        {
                            phase = 4;
                        }
                    }
                    else // destXY2 좌하향
                    {
                        ang -= 1;
                        if (abs(ang) >= getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2))
                        {
                            phase = 4;
                        }
                    }
                }
                else // right obstacle
                {
                    // destXY2 우상향
                    if (ang+270 > abs(temp)) {
                        ang -= 1;
                        if (abs(ang) <= abs(getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2)))
                        {
                            phase = 4;
                        }
                    }
                    else // destXY2 좌하향
                    {
                        ang += 1;
                        if (abs(ang) <= abs(getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2)))
                        {
                            phase = 4;
                        }
                    }
                }
                

                angle = acos((carX - destX2) / sqrt((carX - destX2) * (carX - destX2) + (carY - destY2) * (carY - destY2)));
                if (angle < 0) angle *= -1;
                FInalAngle = angle + AngleError * PIE / 180;
                


                break;
            case 4:
                car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
                if (checkPolygonCircleCollision(carCorners, destX2, destY2, 1))
                {
                    // 충돌 발생!
                    g_bIsGameOver = true;
                    CollisionCount++;
                    //MessageBox(hWnd, TEXT("충돌!!"), TEXT("게임 오버"), MB_OK | MB_ICONSTOP);
                    //PostQuitMessage(0);
                }
                if (carX < destX2 && phase == 4) // 장애물이 오른쪽
                {
                    carX += 2 * cos(FInalAngle);
                }
                if (carX > destX2 && phase == 4) // 장애물이 왼쪽일 때
                {
                    carX -= 2 * cos(FInalAngle);
                }
                if (carY > destY2 && phase == 4)
                {
                    carY -= 2 * sin(FInalAngle);
                    if (carY <= destY2)
                    {
                        phase = 5;
                        break;
                    }
                }
                else
                {
                    carY += 2 * sin(FInalAngle);
                    if (carY >= destY2)
                    {
                        phase = 5;
                        break;
                    }
                }
                break;
            case 5:
                
                ang = 0;
                carX = WINSIZEX / 2; carY=WINSIZEY/2;
                car = MakeRectCenter(carX, carY, carSizeX, carSizeY);
                phase = 0;
                break;
            case 10:
                if (outFile.is_open())
                {

                    outFile << "predict1" << predictX1 << "\t"<< predictY1 << "\n";
                    outFile << "predict2" << predictX2 << "\t" << predictY2 << "\n";
                    
                    outFile.close();
                }
                break;
            default:
                break;
            }
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            InvalidateRect(hWnd, NULL, TRUE);
        }
        break;
    }
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            // TODO: 여기에 hdc를 사용하는 그리기 코드를 추가합니다
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////

                                    // render main //
            DrawEllipse(hdc, obstacle);
            DrawCircle(hdc, destX1, destY1, 2);
            DrawCircle(hdc, destX2, destY2, 2);
            switch (phase)
            {
            case 0:
                
                Polygon(hdc, carCorners, 4);

                break;
            case 1:

                //DrawRect(hdc, car);
                DrawEllipse(hdc, obstacle);
                
                getRotatedRectCorners(car, ang, carCorners);
                Polygon(hdc, carCorners, 4);
                
                break;
            case 2:
                //DrawRect(hdc, car);
                //line 돌리기
                // carX, carY 를 기준으로 car.left, car.right, car.top, car.bottom 값을 바꾸기
                // 각도는 아까 저장해둔
                // 선 네개 만들기 line(car.left, car.top) line(car.top, car.right) line(car.right, car.bottom) line(car.bottom, car.left)
                
                //line 돌리기
                getRotatedRectCorners(car, ang, carCorners);
                Polygon(hdc, carCorners, 4);
                break;
            case 3:
                getRotatedRectCorners(car, ang, carCorners);
                Polygon(hdc, carCorners, 4);
                break;

            case 4:
                getRotatedRectCorners(car, ang, carCorners);
                Polygon(hdc, carCorners, 4);
                break;
            case 5:
                getRotatedRectCorners(car, ang, carCorners);
                Polygon(hdc, carCorners, 4);
                break;
            default:
                break;
            }
            
            // for Debug
            DrawDebugText(hdc, 1, 1, L"phase: %d", phase);
            DrawDebugText(hdc, 1, 21, L"predictX1: %.2f", predictX1);
            //DrawDebugText(hdc, 1, 51, L"angle between: %.2f", getAngleBetween(carX, carY, carX, carY - 10, destX1, destY1) - getAngleBetween(carX, carY, carX, carY - 10, destX2, destY2));
            DrawDebugText(hdc, 1, 41, L"Angle: %.2f", abs(ang));
            DrawDebugText(hdc, 1, 61, L"MotorAngleError: %.2f", AngleError);
            DrawDebugText(hdc, 1, 81, L"CollisionCount: %d", CollisionCount);
            DrawDebugText(hdc, 1, 101, L"angle: %.2f", ang);
            DrawDebugText(hdc, 1, 121, L"temp: %.2f", temp);
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////            
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// 정보 대화 상자의 메시지 처리기입니다.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}


void getRotatedRectCorners(const RECT& rect, double thetaDeg, POINT outCorners[4]) {
    // 1. RECT의 중심점을 double형으로 계산
    double centerX = (rect.left + rect.right) / 2.0;
    double centerY = (rect.top + rect.bottom) / 2.0;

    // 2. 원본 RECT의 4개 꼭짓점 좌표 (double형)
    //    (Polygon 순서: 좌상 -> 우상 -> 우하 -> 좌하)
    double original_x[4] = { (double)rect.left, (double)rect.right, (double)rect.right, (double)rect.left };
    double original_y[4] = { (double)rect.top, (double)rect.top, (double)rect.bottom, (double)rect.bottom };

    // 3. 각도를 라디안으로 변환
    double thetaRad = thetaDeg * (PIE / 180.0);
    double s = std::sin(thetaRad);
    double c = std::cos(thetaRad);

    // 4. 4개의 꼭짓점을 순회하며 회전 변환 적용
    for (int i = 0; i < 4; ++i) {
        // 4-1. 중심점 기준으로 평행 이동
        double x = original_x[i] - centerX;
        double y = original_y[i] - centerY;

        // 4-2. 2D 회전 행렬 적용
        double x_new = (x * c) - (y * s);
        double y_new = (x * s) + (y * c);

        // 4-3. 다시 원래 중심으로 평행 이동
        double final_x = x_new + centerX;
        double final_y = y_new + centerY;

        // 4-4. POINT 배열에 long 타입으로 변환(반올림)하여 저장
        outCorners[i].x = static_cast<long>(final_x + 0.5);
        outCorners[i].y = static_cast<long>(final_y + 0.5);
    }
}

double getClosestDistSqToSegment(double pX, double pY,
    double seg1X, double seg1Y,
    double seg2X, double seg2Y) {

    double segLenX = seg2X - seg1X;
    double segLenY = seg2Y - seg1Y;
    double segLengthSq = (segLenX * segLenX) + (segLenY * segLenY);

    // 1. 선분의 길이가 0인가? (seg1과 seg2가 같은 점)
    if (segLengthSq == 0.0) {
        double dx = pX - seg1X;
        double dy = pY - seg1Y;
        return (dx * dx) + (dy * dy);
    }

    // 2. 선분 상의 최단 지점 '비율(t)'을 계산 (내적 사용)
    //    t = [(P - A) · (B - A)] / |B - A|^2
    double t = ((pX - seg1X) * segLenX + (pY - seg1Y) * segLenY) / segLengthSq;

    // 3. 비율(t)을 0.0 ~ 1.0 사이로 '클램핑(clamping)'
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;

    // 4. 선분 위의 가장 가까운 점(closestX, closestY) 좌표 계산
    double closestX = seg1X + t * segLenX;
    double closestY = seg1Y + t * segLenY;

    // 5. 점에서 그 '가장 가까운 점'까지의 거리 제곱 반환
    double dx = pX - closestX;
    double dy = pY - closestY;
    return (dx * dx) + (dy * dy);
}

bool isPointInPolygon(double testX, double testY, const POINT polygon[4]) {
    bool isInside = false;

    // Ray Casting Algorithm (짝/홀 검사)
    // 다각형의 각 변을 순회 (p0-p1, p1-p2, p2-p3, p3-p0)
    for (int i = 0, j = 3; i < 4; j = i++) {
        // (POINT의 long을 double로 캐스팅)
        double p_i_x = (double)polygon[i].x;
        double p_i_y = (double)polygon[i].y;
        double p_j_x = (double)polygon[j].x;
        double p_j_y = (double)polygon[j].y;

        // 1. 점의 Y좌표가 현재 변의 Y 범위 내에 있는가?
        bool yCheck = (p_i_y > testY) != (p_j_y > testY);
        // 2. 점이 변의 왼쪽에 있는가? (교차 판정)
        bool xCheck = (testX < (p_j_x - p_i_x) * (testY - p_i_y) / (p_j_y - p_i_y) + p_i_x);

        if (yCheck && xCheck) {
            isInside = !isInside; // 교차 횟수(홀/짝) 뒤집기
        }
    }
    return isInside;
}

bool checkPolygonCircleCollision(const POINT polygon[4], double circleX, double circleY, double circleRadius)
{
    // 1. (중심점 내부 판정)
    //    원의 중심이 다각형 내부에 있으면 무조건 충돌입니다.
    //    (헬퍼 함수 2 호출)
    if (isPointInPolygon(circleX, circleY, polygon)) {
        return true;
    }

    // 2. (거리 판정)
    //    원의 중심이 다각형 외부에 있는 경우,
    //    다각형의 '변' 중 하나라도 원의 반지름 이내로 가까우면 충돌입니다.

    double radiusSquared = circleRadius * circleRadius;

    // 다각형의 4개 변을 모두 순회 (p0->p1, p1->p2, p2->p3, p3->p0)
    for (int i = 0, j = 3; i < 4; j = i++) {

        // j: 이전 꼭짓점, i: 현재 꼭짓점
        double seg1X = (double)polygon[j].x;
        double seg1Y = (double)polygon[j].y;
        double seg2X = (double)polygon[i].x;
        double seg2Y = (double)polygon[i].y;

        // 원의 중심에서 (seg1-seg2) 변까지의 최단 거리(의 제곱)를 구함
        // (헬퍼 함수 1 호출)
        double distSq = getClosestDistSqToSegment(circleX, circleY,
            seg1X, seg1Y, seg2X, seg2Y);

        // 3. (최종 판정)
        //    그 거리(의 제곱)가 반지름(의 제곱)보다 작거나 같으면 충돌입니다.
        if (distSq <= radiusSquared) {
            return true;
        }
    }

    // 위 1, 2번을 모두 통과했다면 충돌하지 않은 것입니다.
    return false;
}
