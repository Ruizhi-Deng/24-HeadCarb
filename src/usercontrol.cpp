#include "vex.h"

#include "controller.h"
#include "parameters.h"
#include "basic-functions.h"
#include "chassis.h"
#include "positionNew.h"
#include "usercontrol.h"
#include "autonomous.h"
#include <iostream>
#include <mutex>
using namespace std;
using namespace vex;

bool isUsrCtl = true, last_isUsrCtl = true;
mutex m;

/**
 * @brief 底盘车头控制
 *
 */
void baseControl()
{
    static bool isreversed = false;
    double a1, a3, a4;
    a1 = (abs(A1) < 3) ? 0 : A1;
    a3 = (abs(A3) < 3) ? 0 : A3;
    a4 = (abs(A4) < 30) ? 0 : A4;
    if (press_A)
    {
        press_A = false;
        isreversed = !isreversed;
        if (isreversed)
        {
            Controller.rumble("-");
        }
        if (!isreversed)
        {
            Controller.rumble(".");
        }
    }
    if (isreversed)
    {
        a3 = -a3;
    }
    Chassis::getInstance()->manualSetRobotVel(a3, a1 * 0.4);
    // Chassis::getInstance()->setStopBrakeType(hold);
}

void pistonControl()
{
    static bool pistonL = false;
    static bool pistonR = false;
    static bool pistonF = false;
    if (press_L2)
    {
        press_L2 = false;
        pistonL = !pistonL;
        setPistonLB(pistonL);
    }
    if (press_L1)
    {
        press_L1 = false;
        pistonR = !pistonR;
        // setPistonRB(pistonR);
        setPistonF(pistonR);
    }
}

void lifterControl()
{
    if (X)
    {
        moveLifter(100);
    }
    else if (B)
    {
        moveLifter(-100);
    }
    else
    {
        moveLifter(0);
    }
}

void intakerControl()
{
    if (R1)
    {
        moveIntaker(100);
    }
    else if (R2)
    {
        moveIntaker(-100);
    }
    else
    {
        moveIntaker(0);
    }
}

void usrCtlThread(void *childThread)
{
    vex::thread *Thread = (vex::thread *)childThread;
    vex::thread *T = NULL;
    while (true)
    {
        // if(isUsrCtl && !last_isUsrCtl)
        // {
        //     //重置所有外设

        //     //中断进程
        //     Thread->interrupt();
        //     Chassis::getInstance()->autoSetRobotVel(Vector(0, 0), 0);
        // }

        /*---------技能赛手动自动切换 ———— 手动 -> 自动：按LEFT键 / 自动 -> 手动：按X&&A键--------------*/

        if (press_LEFT && T == NULL)
        { // 进入自动
            isUsrCtl = !isUsrCtl;
            press_LEFT = false;
            // cout << "LEFT Pressed" << endl;
            T = new thread(autonomous);
            // cout << "111" << endl;
        }
        if (DOWN && T != NULL)
        { // 中断自动进手动
            isUsrCtl = !isUsrCtl;
            T->interrupt();
            delete T;
            T = NULL;
            Chassis::getInstance()->autoSetRobotVel(0, 0);
        }

        /*---------------------------------------------------------------------------------------*/

        if (isUsrCtl)
        {
            static bool brakeType = false;
            // 底盘控制
            baseControl();
            pistonControl();
            intakerControl();
            lifterControl();
            // 其余组件控制

            if (Y)
            {
                Y = false;
                Inertial.resetHeading();
                Position::getInstance()->setHeading(0);
                Position::getInstance()->setGlobalPosition(0, 0);
                // GPS.setOrigin();
                // GPS.resetHeading();
                // GPS.setLocation(0, 0, 0);
            }

            // cout << "deg: " << getPuncherPos() << endl;
            Point pos = Position::getInstance()->getPos();
            // cout << tmpPuncherPos << " " << getPuncherPos() << endl;
            Controller.Screen.setCursor(1, 1);
            Controller.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
            // Controller.Screen.setCursor(5, 1);
            // Brain.Screen.setCursor(2, 1);
            // Brain.Screen.print("%.3lf %.3lf %.3lf", Position::getInstance()->sensorX, Position::getInstance()->sensorY, Position::getInstance()->sensorHeading);
            // Brain.Screen.setCursor(3, 1);
            // Brain.Screen.print("%.3f", IMUHeading());
            // Brain.Screen.setCursor(4, 1);
            // Brain.Screen.print("%.3f", Position::getInstance()->getInitHeading());
            // Brain.Screen.setCursor(5, 1);
            // Brain.Screen.print("%.3f", Position::getInstance()->getSensorHeading());
            // 底盘锁定模式切换
            if (RIGHT)
            {
                brakeType = !brakeType;
                if (brakeType)
                {
                    Chassis::getInstance()->chassisBrake(brakeType::hold);
                    Chassis::getInstance()->setStopBrakeType(brakeType::hold);
                }
                else
                {
                    Chassis::getInstance()->chassisBrake(brakeType::coast);
                    Chassis::getInstance()->setStopBrakeType(brakeType::coast);
                }
                RIGHT = false;
            }
        }
        this_thread::sleep_for(RefreshTime);
    }
}

void autoCtlThread()
{
    // 单状态机实现，状态变量为全局变量，每一个状态完成一个非阻塞自动函数
    //  while(isUsrCtl){
    //      cout << "isUsrCtl" << endl;
    //      this_thread::sleep_for(1000);
    //  }
    //  cout << "auto" << endl;
    //  autonomous();
    return;
}

void usercontrol()
{
    Chassis::getInstance()->autoSetRobotVel(0, 0);
    cout << "enter user control" << endl;

    thread AutoCtl(autoCtlThread);
    // thread UsrCtl(usrCtlThread, &AutoCtl);
    usrCtlThread(&AutoCtl);
    // UsrCtl.join();
    // while (true)
    // {
    //     this_thread::sleep_for(10);
    //     // Controller.Screen.setCursor(5, 1);
    //     // Controller.Screen.print(isUsrCtl? "USR" : "AUTO");
    // }
}
