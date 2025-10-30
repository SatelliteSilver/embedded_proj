#include "utils.h"

double getSlope(double xcenter, double ycenter, double destX, double destY) {

    // 1. y의 변화량 (Rise) 계산
    double deltaY = destY - ycenter;

    // 2. x의 변화량 (Run) 계산
    double deltaX = destX - xcenter;

    // --- 엣지 케이스 처리 ---

    // 3. (Edge Case 1) x의 변화량이 0일 때 (분모가 0)
    if (deltaX == 0.0) {

        // 3-1. y의 변화량도 0이면 (0 / 0) => 두 점이 같은 위치
        if (deltaY == 0.0) {
            // 수학적으로 기울기가 '정의되지 않음(Indeterminate)'
            return std::numeric_limits<double>::quiet_NaN(); // "Not a Number"
        }

        // 3-2. y의 변화량이 0이 아니면 (N / 0) => 수직선
        // 수학적으로 기울기가 '무한대(Undefined as a real number)'
        // (deltaY가 양수면 +inf, 음수면 -inf)
        return std::numeric_limits<double>::infinity() * (deltaY > 0 ? 1 : -1);
    }

    // 4. (Normal Case) 일반적인 기울기 계산
    double L = deltaY / deltaX;

    return L;
}

double getAngleBetween(double xc, double yc,double x1, double y1,double x2, double y2) {

    // 1. 센터에서 각 점을 향하는 두 개의 벡터(Vector)를 계산합니다.
    double v1x = x1 - xc;
    double v1y = y1 - yc;

    double v2x = x2 - xc;
    double v2y = y2 - yc;

    // 2. 두 벡터의 '내적(Dot Product)'을 계산합니다.
    // 내적 = (v1x * v2x) + (v1y * v2y)
    double dotProduct = (v1x * v2x) + (v1y * v2y);

    // 3. 각 벡터의 크기(길이)를 계산합니다. (피타고라스 정리)
    double magV1 = std::sqrt(v1x * v1x + v1y * v1y);
    double magV2 = std::sqrt(v2x * v2x + v2y * v2y);

    // 4. 엣지 케이스 처리: 벡터 길이가 0이면 (점이 센터와 겹침)
    // 0으로 나누기 오류를 방지합니다.
    if (magV1 == 0.0 || magV2 == 0.0) {
        return 0.0; // 사잇각을 0으로 처리
    }

    // 5. cos(각도) 값을 계산합니다. (공식: 내적 / (크기1 * 크기2))
    double cosTheta = dotProduct / (magV1 * magV2);

    // 6. 부동 소수점 오류 보정 (cosTheta 값은 -1.0 ~ 1.0 사이여야 함)
    // 계산 오류로 1.0000001 같은 값이 나오면 acos가 NaN을 반환합니다.
    if (cosTheta > 1.0) {
        cosTheta = 1.0;
    }
    else if (cosTheta < -1.0) {
        cosTheta = -1.0;
    }

    // 7. acos (아크코사인)을 사용해 라디안 각도를 구합니다.
    double angleRad = std::acos(cosTheta);

    // 8. 라디안을 '도(Degree)'로 변환하여 반환합니다.
    const double PI = std::acos(-1.0); // PI 값
    return angleRad * (180.0 / PI);
}

double getDiagonal(double side1, double side2) {
    // 피타고라스의 정리: c = sqrt(a^2 + b^2)
    // std::hypot(side1, side2) 함수를 사용하면 더 안정적으로 계산할 수도 있습니다.
    return std::sqrt((side1 * side1) + (side2 * side2));
}

