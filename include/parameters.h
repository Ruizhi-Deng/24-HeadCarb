#ifndef PARAMETERS_H_
#define PARAMETERS_H_
#include "calc.h"
#include "vex.h"
#include "geometry.h"
#include <math.h>

const double PI = M_PI;
const float WheelRadius = 3.492;         // 主动轮半径 cm
const float TrackingWheelRadius = 3.492; // 定位轮半径 cm

const double IMUCoefficient = 3600 / 3594;

// const double targetPosX = 283;
// const double targetPosY = 20;

const double targetPosX = 41.23;
const double targetPosY = 279.26;

const float ControlCycle = 10;
const float RefreshTime = 10;        // 刷新时间 ms
const float positionRefreshTime = 3; // 定位刷新时间 ms

const int CENTER_FOV = 150;

const double XOffsetFromCenter = 0;
const double YOffsetFromCenter = -7.6;
const Vector OffsetFromCenter = Vector(XOffsetFromCenter, YOffsetFromCenter);

// centre at 158

// #define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE

// #ifdef RED_ALLIANCE
// const float color_range[2] = {340, 50};
// #define VisionSensor__SIG_TAR VisionSensor__SIG_RED
// #endif

// #ifdef BLUE_ALLIANCE
// const float color_range[2] = {190, 280};
// #define VisionSensor__SIG_TAR VisionSensor__SIG_BLUE
// #endif

#endif