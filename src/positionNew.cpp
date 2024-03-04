#include "positionNew.h"
#include <cmath>
#include "my-timer.h"
#include "basic-functions.h"
#include "parameters.h"
#include "geometry.h"
#include <stdlib.h>
#include <iomanip>
#include <iostream>
using namespace std;

Position::Position()
{
    vexGenericSerialEnable(ENCODERPORT, 0);
    vexGenericSerialBaudrate(ENCODERPORT, 115200);
    sampleTime = 0;
    Timer.reset();
    lastTime = Timer.getTimeDouble();
    lastGlobalX = lastGlobalY = lastIMUHeading = IMUHeading = 0;
    lastSensorX = lastSensorY = lastSensorHeading = 0;
    globalYSpeed = globalXSpeed = 0;
    globalY = globalX = 0;
    for (int i = 0; i < 32; ++i)
        data[i] = 0;
    for (int i = 1; i <= 5; ++i)
        getData();
    initHeading = sensorHeading + 180;
    while (initHeading < -180)
        initHeading += 360;
    while (initHeading > 180)
        initHeading -= 360;
    initX = sensorX / 10;
    initY = sensorY / 10;
    initXOffset = 0;
    initYOffset = 0;
}

void Position::getData()
{
    // uint8_t dt[2] = 0;
    // Serial1.read(dt,1);
    // while(dt[0] != 0x0D){
    //     Serial1.read(dt,1);
    // }
    lastSensorX = sensorX;
    lastSensorY = sensorY;
    lastSensorHeading = sensorHeading;
    uint8_t data2[32];
    while (vexGenericSerialPeekChar(ENCODERPORT) != -1)
    {
        // cout << "available"<< endl;
        for (int i = 31; i > 0; --i)
        {
            data[i] = data[i - 1];
        }
        data[0] = vexGenericSerialReadChar(ENCODERPORT);
        // cout << "data: "<< data[0] << endl;
        if (data[27] == 0x0D && data[26] == 0x0A && data[1] == 0x0A && data[0] == 0x0D)
        {
            for (int i = 0; i < 28; ++i)
            {
                data2[i + 2] = data[27 - i];
            }
            break;
        }
    }
    float dt[7];
    for (int i = 1; i < 7; ++i)
    {
        dt[i] = ((float *)(data2))[i];
    }
    sensorX = dt[4];
    sensorY = dt[5];
    sensorHeading = dt[1];
    if (abs(sensorHeading) < 0.005)
        sensorHeading = lastSensorHeading;
    if (abs(sensorX) < 0.005)
        sensorX = lastSensorX;
    if (abs(sensorY) < 0.005)
        sensorY = lastSensorY;
}

void Position::updateGlobalPos()
{
    lastGlobalX = globalX;
    lastGlobalY = globalY;
    lastIMUHeading = IMUHeading;

    // 定位轮交点世界坐标（世界坐标以开机位置为准）
    double tempX = sensorX / 10;
    double tempY = sensorY / 10;
    // 定位轮交点位场地坐标（场地坐标以进程序位置为准）
    globalX = (tempX - initX) * cos(deg2rad(-initHeading)) - (tempY - initY) * sin(deg2rad(-initHeading)) + initXOffset;
    globalY = (tempX - initX) * sin(deg2rad(-initHeading)) + (tempY - initY) * cos(deg2rad(-initHeading)) + initYOffset;

    // globalX = sensorX;
    // globalY = sensorY;

    // cout<<globalX<<"    "<<globalY<<endl;

    // IMUHeading = sensorHeading;
    // if (abs(sensorHeading) > 0.005){
    double tempHead = sensorHeading - initHeading;
    // double tempHead = sensorHeading - initHeading;
    while (tempHead <= -180)
        tempHead += 360;
    while (tempHead > 180)
        tempHead -= 360;
    tempHead = -tempHead + 180;
    while (tempHead < 0)
        tempHead += 360;
    while (tempHead >= 360)
        tempHead -= 360;
    // IMUHeading = abs(tempHead) > 0.005 ? tempHead : lastIMUHeading;
    IMUHeading = tempHead;
    // }
    // IMUHeading = 360 - abs(IMUHeading) > 0.005 ? IMUHeading : lastIMUHeading;

    // cout<< globalX <<" "<< globalY << " " << IMUHeading<<endl;
    // cout<<IMUHeading<<endl;
}

void Position::updateGlobalSpeed()
{
    sampleTime = (Timer.getTimeDouble() - lastTime) * 1000;
    lastTime = Timer.getTimeDouble();
    globalXSpeed = (globalX - lastGlobalX) / sampleTime;
    globalYSpeed = (globalY - lastGlobalY) / sampleTime;
}

/**
 * @brief 更新世界坐标系下机器位置（每tick调用）
 *
 */
void Position::updatePos()
{
    getData();
    updateGlobalPos();
    updateGlobalSpeed();
}

/**
 * @brief 获取当前旋转中心于场地坐标系的位置
 *
 * @return Point
 */
Point Position::getPos() const
{
    return Point(globalX, globalY) - OffsetFromCenter + OffsetFromCenter.rotateTrans(-IMUHeading);
}

double Position::getXSpeed() const
{
    return globalXSpeed;
}

double Position::getYSpeed() const
{
    return globalYSpeed;
}

double Position::getIMUHeading() const
{
    return IMUHeading;
}

double Position::getInitHeading() const
{
    return initHeading;
}

void Position::resetXPosition()
{
    // Vector tmp(sensorX, sensorY);
    // tmp.rotateTrans(-initHeading);
    // Xfix = 0 - tmp._x;
    // Xfix = initX;
    // globalX = 0;
}

void Position::resetYPosition()
{
    // Vector tmp(sensorX, sensorY);
    // tmp.rotateTrans(-initHeading);
    // Yfix = 0 - tmp._y;
    // Yfix = initY;
    // globalY = 0;
}

void Position::setGlobalPosition(double _x, double _y)
{
    // Xfix = sensorX / 20 - (_x * cos(initHeading) + _y * sin(initHeading));
    // Yfix = sensorY / 20 - (_y * cos(initHeading) - _x * sin(initHeading));
    // globalX = _x;
    // globalY = _y;

    initHeading = sensorHeading + 180;
    while (initHeading < -180)
        initHeading += 360;
    while (initHeading > 180)
        initHeading -= 360;
    initX = sensorX / 10;
    initY = sensorY / 10;
    initXOffset = _x;
    initYOffset = _y;
}

void Position::setHeading(double head)
{
    if (abs(sensorHeading) > 0.005)
    {
        initHeading = sensorHeading - head + 180;
        while (initHeading < -180)
            initHeading += 360;
        while (initHeading > 180)
            initHeading -= 360;
    }
}

double Position::getSensorHeading() const
{
    return sensorHeading;
}

void updatePosition()
{
    while (true)
    {
        Position::getInstance()->updatePos();
        this_thread::sleep_for(positionRefreshTime);
    }
}