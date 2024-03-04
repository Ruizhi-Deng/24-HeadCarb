#include "debugger.h"
#include "basic-functions.h"
#include "robot-config.h"
#include "geometry.h"
#include "usercontrol.h"
#include "positionNew.h"

#include "parameters.h"
#include "controller.h"
#include "auto-functions.h"
#include <iostream>
using namespace std;


void debugControl(){
    enum STATE{
        USR=1, TESTPID
    }state;

    //debug需要的局部变量
    state = USR;
    Point pos;
    Position *p = Position::getInstance();

    while (true)
    {
        switch (state)
        {
            //手动控制【1】
            case USR:
                baseControl();
                pos = Position::getInstance()->getPos();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("%.3lf %.3lf %.3lf", pos._x, pos._y, IMUHeading());
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("%.3lf %.3lf %.3lf", Position::getInstance()->sensorX, Position::getInstance()->sensorY, Position::getInstance()->sensorHeading);
                Brain.Screen.setCursor(3, 1);
                Brain.Screen.print("%.3f", IMUHeading());
                Brain.Screen.setCursor(4, 1);
                Brain.Screen.print("%.3f", Position::getInstance()->getInitHeading());
                Brain.Screen.setCursor(5, 1);
                Brain.Screen.print("%.3f", Position::getInstance()->getSensorHeading());
                
                if(press_B)
                {
                    press_B = false;
                    Position::getInstance()->setGlobalPosition(0, 0);
                    setIMUHeading(0);
                    clearBrainScr();
                    clearControllerScr();
                    
                }
                
                if (press_DOWN) 
                {
                    press_DOWN = false;
                    state = TESTPID;
                    clearBrainScr();
                    clearControllerScr();
                }
                if (press_UP) 
                {
                    press_UP = false;
                    state = TESTPID;
                    clearBrainScr();
                    clearControllerScr();
                    Controller.Screen.setCursor(5, 1);
                }
                break;
            //调试PID【2】
            case TESTPID:
                // baseControlbyHeading();
                
                pos = Position::getInstance()->getPos();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("%lf %lf %lf", pos._x, pos._y, IMUHeading());
                if(press_A)
                {
                    press_A = false;
                    turnTo(90, 80);
                    cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;

                }

                if(press_Y)
                {
                    press_Y = false;
                    turnTo(0, 80);
                    cout << p->getPos()._x << " " << p->getPos()._y << " " << IMUHeading() << endl;

                }

                if(press_B)
                {
                    Position::getInstance()->setGlobalPosition(0, 0);
                    setIMUHeading(0);
                    press_B = false;
                }

                if(press_DOWN) 
                {
                    press_DOWN = false;
                    state = USR;
                    // Brain.Screen.clearScreen();
                    clearBrainScr();
                    clearControllerScr();
                }
                if(press_UP) 
                {
                    press_UP = false;
                    state = USR;
                    clearBrainScr();
                    clearControllerScr();
                }
                break;
        }

        Controller.Screen.setCursor(5, 1);
        Controller.Screen.print("%d", state);
        this_thread::sleep_for(RefreshTime);
    }
}

// #ifdef debug
// // cout<< Position::getInstance()->TrackingWheelLSpeed() << endl;
// std::cout << pos._x << " " << pos._y << " " << IMUHeading() << endl;
// if (UP && !last_UP){
//     Position *p = Position::getInstance();
//     // cout << p->getPos()._x << ", " << p->getPos()._y << ", " << IMUHeading() << endl;
// }
// if (DOWN && !last_DOWN){
//         PID adjPID;
//         adjPID.setCoefficient(0.6, 0, 0);
//         adjPID.setDTolerance(5);
//         adjPID.setErrorTolerance(2);
//         adjPID.setIMax(30);
//         adjPID.setIRange(5);
//         adjPID.setJumpTime(300);

//         // Inertial.setHeading(180, rotationUnits::deg);
//         // AutoPIDAdjustment(adjPID, 20, adjustGetPos, adjustForward, RefreshTime);
//         AutoPIDAdjustment(adjPID, 20, inertialHeading, moveClockwise, RefreshTime);
// }
// #endif