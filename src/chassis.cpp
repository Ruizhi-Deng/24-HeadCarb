#include "vex.h"
#include "chassis.h"
#include "parameters.h"
#include "robot-config.h"
#include <cmath>
#include <iostream>
using namespace vex;
using namespace std;

/// @brief 底盘构造函数
Chassis::Chassis()
{
    robotVel = 0;
    autoVel = 0;
    manualVel = 0;
    stopBrakeType = coast;
    robotVelR = 0;
    manualVelR = 0;
    autoVelR = 0;
}

/// @brief 【换算函数】将机器人坐标系下的速度转换为左右侧轮子速度
void Chassis::calcWheelVel()
{
    double robotSpeed = abs(robotVel);
    double maxAbsVel = 0;
    double maxAbsWheelVel = 100;

    wheelVelL = robotVel + robotVelR;
    wheelVelR = robotVel - robotVelR;
    wheelVelL = wheelVelL / maxAbsWheelVel * 100;
    wheelVelR = wheelVelR / maxAbsWheelVel * 100;
}

/// @brief 【换算函数】将轮速转换为对应电机电压
void Chassis::calcWheelVolt()
{
    wheelVoltL = wheelVelL * speedGain;
    if (abs(wheelVoltL) < 1000)
        wheelVoltL = 0;
    wheelVoltR = wheelVelR * speedGain;
    if (abs(wheelVoltR) < 1000)
        wheelVoltR = 0;
}

/// @brief 按指定电压驱动电机
void Chassis::setMotorVolt()
{
    double leff = 1;
    double reff = 0.98;
    if (wheelVoltL == 0)
    {
        Motor_BaseL1.stop(stopBrakeType);
        Motor_BaseL2.stop(stopBrakeType);
        Motor_BaseL3.stop(stopBrakeType);
        Motor_BaseL4.stop(stopBrakeType);
        Motor_BaseL5.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseL1.spin(directionType::fwd, wheelVoltL * leff, voltageUnits::mV);
        Motor_BaseL2.spin(directionType::fwd, wheelVoltL * leff, voltageUnits::mV);
        Motor_BaseL3.spin(directionType::fwd, wheelVoltL * leff, voltageUnits::mV);
        Motor_BaseL4.spin(directionType::fwd, wheelVoltL * leff, voltageUnits::mV);
        Motor_BaseL5.spin(directionType::fwd, wheelVoltL * leff, voltageUnits::mV);
    }

    if (wheelVoltR == 0)
    {
        Motor_BaseR1.stop(stopBrakeType);
        Motor_BaseR2.stop(stopBrakeType);
        Motor_BaseR3.stop(stopBrakeType);
        Motor_BaseR4.stop(stopBrakeType);
        Motor_BaseR5.stop(stopBrakeType);
    }
    else
    {
        Motor_BaseR1.spin(directionType::fwd, wheelVoltR * reff, voltageUnits::mV);
        Motor_BaseR2.spin(directionType::fwd, wheelVoltR * reff, voltageUnits::mV);
        Motor_BaseR3.spin(directionType::fwd, wheelVoltR * reff, voltageUnits::mV);
        Motor_BaseR4.spin(directionType::fwd, wheelVoltR * reff, voltageUnits::mV);
        Motor_BaseR5.spin(directionType::fwd, wheelVoltR * reff, voltageUnits::mV);
    }
}

/// @brief 强制底盘刹车（注意会收到电机驱动线程的影响，在使用该函数时确保不与电机驱动线程冲突）
/// @param brake 底盘刹车类型（coast，brake，hold）
void Chassis::chassisBrake(brakeType brake)
{
    wheelVoltL = 0;
    wheelVoltR = 0;

    Motor_BaseR1.stop(brakeType::brake);
    Motor_BaseR2.stop(brakeType::brake);
    Motor_BaseR3.stop(brakeType::brake);
    Motor_BaseR4.stop(brakeType::brake);
    Motor_BaseR5.stop(brakeType::brake);

    Motor_BaseL1.stop(brakeType::brake);
    Motor_BaseL2.stop(brakeType::brake);
    Motor_BaseL3.stop(brakeType::brake);
    Motor_BaseL4.stop(brakeType::brake);
    Motor_BaseL5.stop(brakeType::brake);
}

/// @brief 底盘电机驱动函数，将作为单独线程不断刷新
void Chassis::chassisRun()
{
    setMotorVolt();
}

/// @brief 【手动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度，速度大小在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::manualSetRobotVel(double Vel, double VelR)
{
    if (abs(Vel) > 100)
    {
        Vel = Vel / abs(Vel) * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    manualVel = Vel;
    manualVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (abs(robotVel) > 100)
    {
        robotVel = robotVel / abs(robotVel) * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/// @brief 【自动控制】通过机器坐标系速度驱动底盘
/// @param Vel 机器坐标系下速度，速度大小在 0-100 之间
/// @param VelR 机器人角速度，范围在-100 - 100
void Chassis::autoSetRobotVel(double Vel, double VelR)
{
    if (abs(Vel) > 100)
    {
        Vel = Vel / abs(Vel) * 100;
    }
    if (abs(VelR) > 100)
        VelR = 100 * sign(VelR);
    autoVel = Vel;
    autoVelR = VelR;
    robotVel = manualVel + autoVel;
    robotVelR = manualVelR + autoVelR;
    if (abs(robotVelR) > 100)
    {
        robotVel = robotVel / abs(robotVel) * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

void Chassis::autoSetWheelVel(double Vel_L, double Vel_R)
{
    robotVel = (Vel_L + Vel_R) / 2;
    robotVelR = (Vel_L - Vel_R) / 2;
    if (abs(robotVelR) > 100)
    {
        robotVel = robotVel / abs(robotVel) * 100;
    }
    if (abs(robotVelR) > 100)
        robotVelR = 100 * sign(robotVelR);
    calcWheelVel();
    calcWheelVolt();
}

/**
 * @brief 设置停止时的刹车类型
 *
 * @param brake
 */
void Chassis::setStopBrakeType(brakeType brake)
{
    stopBrakeType = brake;
}

void updateChassis()
{
    while (true)
    {
        Chassis::getInstance()->chassisRun();
        this_thread::sleep_for(RefreshTime);
    }
}
