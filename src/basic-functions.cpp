#include "basic-functions.h"
#include "vex.h"
#include "robot-config.h"
#include "positionNew.h"
#include "chassis.h"
#include <iostream>
using namespace vex;

void delay(int msec) // 等待，单位毫秒
{
    this_thread::sleep_for(msec);
}

void setPistonF(bool state)
{
    if (state)
    {
        Piston_FI.set(false);
        Piston_FO.set(true);
    }
    else
    {
        Piston_FO.set(false);
        Piston_FI.set(true);
    }
}

void setPistonLB(bool state)
{
    if (state)
    {
        Piston_BLI.set(false);
        Piston_BLO.set(true);
    }
    else
    {
        Piston_BLO.set(false);
        Piston_BLI.set(true);
    }
}

void setPistonRB(bool state)
{
    if (state)
    {
        Piston_BRI.set(false);
        Piston_BRO.set(true);
    }
    else
    {
        Piston_BRO.set(false);
        Piston_BRI.set(true);
    }
}

void moveIntaker(float percent)
{
    Motor_Intaker.spin(directionType::fwd, 127 * percent, voltageUnits::mV);
}

void moveIntakerWithRPM(float RPM)
{
    Motor_Intaker.spin(directionType::fwd, RPM, velocityUnits::rpm);
}

void moveLifter(float pct)
{
    Motor_Lifter1.spin(directionType::fwd, 127 * pct, voltageUnits::mV);
    Motor_Lifter2.spin(directionType::fwd, 127 * pct, voltageUnits::mV);
}

// /*????*/
double IMUHeading()
{
    // double heading = Inertial.rotation(rotationUnits::deg);
    // // heading = heading / 3594 * 3600;
    // heading = heading / 3590 * 3600;
    // while (heading < 0) heading += 360;
    // while (heading > 360) heading -= 360;
    // return heading;
    // return Inertial.heading();
    return Position::getInstance()->getIMUHeading();
}

double InitHeading()
{
    // double heading = Inertial.rotation(rotationUnits::deg);
    // // heading = heading / 3594 * 3600;
    // heading = heading / 3590 * 3600;
    // while (heading < 0) heading += 360;
    // while (heading > 360) heading -= 360;
    // return heading;
    return Position::getInstance()->getInitHeading();
}

void setIMUHeading(double deg)
{
    Position::getInstance()->setHeading(deg);
}

void clearBrainScr()
{
    Brain.Screen.clearScreen();
}

void clearControllerScr()
{
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("                                                         ");
}