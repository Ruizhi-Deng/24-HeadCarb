#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "PID.h"
#include "vex.h"
#include "geometry.h"
#include <math.h>
#include "calc.h"


class Chassis
{
private:
    /*----------------------------------------------------------------------------*/
    /*    为了融合手动与自动控制                                                   */ 
    /*    将底盘速度分为手动控制部分与自动控制部分                                  */ 
    /*    二者相加后为底盘控制速度                                                 */
    /*----------------------------------------------------------------------------*/

    //机器人坐标系速度，机器朝向为y轴，机器人朝向的右边为x轴
    double robotVel;

    //机器人角速度
    double robotVelR;

    //机器人坐标系下手动控制的速度
    double manualVel;
    double manualVelR;
    //机器人坐标系下自动控制的速度
    double autoVel;
    double autoVelR;

    double wheelVelL;
    double wheelVelR;
    double wheelVoltL;
    double wheelVoltR;

    vex::brakeType stopBrakeType;

    double speedGain = 127;

    Vector calcRobotVel(Vector Vel);
    void calcWheelVel();
    void calcWheelVolt();
    void setMotorVolt();

    // double updateRobotHeading();

public:
    static Chassis *getInstance(){
        static Chassis *c = NULL;
        if (c == NULL){
            c = new Chassis();
        }
        return c;
    }
    static void deleteInstance(){
        Chassis *c = Chassis::getInstance();
        if(c != NULL){
            delete c;
            c = NULL;
        }
    }
    Chassis();
    void setStopBrakeType(brakeType brake);
    void manualSetRobotVel(double Vel, double VelR);
    void autoSetRobotVel(double Vel, double VelR);
    void autoSetWheelVel(double Vel_L, double Vel_R);
    void chassisBrake(brakeType brake);
    void chassisRun();
};

void updateChassis();


#endif