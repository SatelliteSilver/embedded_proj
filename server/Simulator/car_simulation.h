#pragma once

#include "resource.h"
#include "utils.h"
#include "gdi_utils.h"

void getRotatedRectCorners(const RECT& rect, double thetaDeg, POINT outCorners[4]);
double getClosestDistSqToSegment(double pX, double pY,double seg1X, double seg1Y,double seg2X, double seg2Y);
bool isPointInPolygon(double testX, double testY, const POINT polygon[4]);
bool checkPolygonCircleCollision(const POINT polygon[4], double circleX, double circleY, double circleRadius);
