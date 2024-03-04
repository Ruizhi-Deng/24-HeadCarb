/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       SJTU VEX                                                  */
/*    Created:      Wed Dec 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "chassis.h"
#include "debugger.h"
#include "controller.h"
#include "positionNew.h"
#include "debugger.h"
#include "usercontrol.h"
#include "autonomous.h"
#include <stdlib.h>
#include <iostream>
using namespace std;
using namespace vex;

#ifdef COMPETITION
competition Competition;
#endif

// 定义Competiton会奇妙进入手动程序

int main()
{
    // vexcodeInit();
    cout << "start" << endl;
    Inertial.calibrate();
    while (Inertial.isCalibrating())
    {
        /* code */
    }

#ifdef COMPETITION
    Competition.drivercontrol(usercontrol);
    Competition.autonomous(autonomous);
#endif

#ifdef debug
    thread DebugControl(debugControl);
#endif

    thread UpdatePos(updatePosition);
    thread UpdateChassis(updateChassis);
    thread TController(defineController);
    while (true)
    {
        this_thread::sleep_for(10);
    }
}
