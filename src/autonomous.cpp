#include "autonomous.h"
#include "GPS.h"
#include "my-timer.h"
#include "parameters.h"
#include "robot-config.h"

void autonInit(void) {
    setPistonE1(false);
    setPistonHook(false);
    resetHeading();
    resetForwardPos();
    MyGps.resetForwardPosGps();
}

void auton_test() {
    MyTimer autotimer;
    autotimer.reset();
}

void auton_near_1() {
    MyTimer autotimer;
    autotimer.reset();
    setIntakeSpeed(100);
    setCataStatus(5, 0);
    softStartTimerForward(0, 80, 200);
    posForwardAbsWithHeading(80, 1000, 0);
    PIDPosForwardAbs(1180);
    PIDPosForwardAbs(670);
    PIDAngleRotateAbs(-135, 2.5, 0.05, 5, 1.5);
    setCataStatus(2, 0); // shoot
    this_thread::sleep_for(200);

    PIDAngleRotateAbs(-160, 2, 0.05, 4);
    PIDPosForwardAbs(500);
    posForwardAbsWithHeading(90, 0, -160);
    setIntakeSpeed(0);
    PIDPosForwardAbs(-600);
    PIDAngleRotateAbs(-100, 2, 0.05, 4);
    setIntakeSpeed(-100);
    posForwardAbsWithHeading(50, 400, -100);
    voltageForward(70);
    this_thread::sleep_for(200);
    lockBase();
    this_thread::sleep_for(50);

    setIntakeSpeed(0);
    PIDPosForwardAbs(0);
    PIDAngleRotateAbs(20, 2, 0.05, 5);
    posForwardAbsWithHeading(50, -1300, 20);
    voltageForward(-10);
    this_thread::sleep_for(400);

    setPistonHook(1);
    this_thread::sleep_for(100);
    PIDAngleRotateAbs(-80, 2, 0.05, 5);
    setPistonHook(0);
    PIDPosForwardAbs(-600);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_near_2() {
    MyTimer autotimer;
    autotimer.reset();
    setIntakeSpeed(0);
    setPistonHook(1);
    this_thread::sleep_for(300);
    PIDAngleRotateAbs(-90, 1.1, 0.05, 5);
    setPistonHook(0);

    PIDPosForwardAbs(630);
    // PIDPosCurveAbs(500,300,5);
    PIDAngleRotateAbs(-45, 0.8, 0.05, 5);
    setIntakeSpeed(-100);
    voltageForward(15);
    this_thread::sleep_for(300);
    voltageForward(70);
    this_thread::sleep_for(400);

    setIntakeSpeed(0);
    setCataStatus(5, 0);
    PIDPosForwardAbs(0);
    PIDAngleRotateAbs(-90, 0.8, 0.05, 5);
    PIDPosForwardAbs(-650);
    PIDAngleRotateAbs(-138, 0.8, 0.05, 5);
    PIDPosForwardAbs(-830);
    // PIDPosCurveAbs(-1600,-1100,5);

    this_thread::sleep_for(100);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_1() {
    MyTimer autotimer;
    autotimer.reset();
    setIntakeSpeed(100);
    setCataStatus(5, 1);
    softStartTimerForward(0, 80, 200);
    posForwardAbsWithHeading(80, 1100, 0);
    PIDPosForwardAbs(1300); // 吸到第一颗球
    setIntakeSpeed(0);
    PIDAngleRotateAbs(105, 2, 0.05, 5); // 转到goal
    setIntakeSpeed(-100);               // 吐球
    // posForwardAbsWithHeading(80, 150, 105); // 送球
    voltageForward(15);          // 送球
    this_thread::sleep_for(200); // 送球
    // setIntakeSpeed(-100);
    voltageForward(70);          // 送球
    this_thread::sleep_for(420); // 送球
    // PIDPosForwardAbs(200);

    PIDPosForwardAbs(0);                   // 第一颗球后退到中间
    PIDAngleRotateAbs(-75, 1.5, 0.05, 10); // 朝向第二颗球
    setIntakeSpeed(100);                   // 吸球
    PIDPosForwardAbs(350);                 // 前进
    voltageForward(40);                    // 吸到第二颗球
    this_thread::sleep_for(300);           // 吸到第二颗球
    setIntakeSpeed(30);                    // 吸球
    PIDPosForwardAbs(50);                  // 第二颗球后退到中间
    PIDAngleRotateAbs(120, 1.5, 0.05, 10); // 朝向goal
    setIntakeSpeed(-100);                  // 吐球
    posForwardAbsWithHeading(80, 150, 120);
    voltageForward(70); // 送球
    this_thread::sleep_for(450);
    // PIDPosForwardAbs(200);

    PIDPosForwardAbs(50);                   // 送完第二颗球后退到中间
    PIDAngleRotateAbs(-165, 1.5, 0.05, 10); // 朝向围板
    PIDPosForwardAbs(450);                  // 前进
    PIDAngleRotateAbs(-75, 1.5, 0.05, 10);  // 朝向第三颗球
    setIntakeSpeed(100);                    // 吸第三颗球
    voltageForward(70);                     // 吸到第三颗球
    this_thread::sleep_for(300);            // 吸到第三颗球
    // PIDPosForwardAbs(350);
    setIntakeSpeed(20);                   // 吸第三颗球
    PIDPosForwardAbs(50);                 // 回到中间
    PIDAngleRotateAbs(78, 1.5, 0.05, 10); // 朝向goal
    setIntakeSpeed(-100);                 // 吐球
    voltageForward(70);                   // 送第三颗球
    this_thread::sleep_for(700);
    PIDPosForwardAbs(0); // 回到中间
    // setPistonE(1);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void auton_far_2() {
    MyTimer autotimer;
    autotimer.reset();
    setCataStatus(5, 1);
    softStartTimerForward(0, 80, 200);
    posForwardAbsWithHeading(80, 1100, 0);
    PIDPosForwardAbs(1300); // 吸到第一颗球
    setIntakeSpeed(0);
    PIDAngleRotateAbs(105, 2, 0.05, 5); // 转到goal
    setIntakeSpeed(-100);               // 吐球
    // posForwardAbsWithHeading(80, 150, 105); // 送球
    voltageForward(15);          // 送球
    this_thread::sleep_for(200); // 送球
    // setIntakeSpeed(-100);
    voltageForward(70);          // 送球
    this_thread::sleep_for(420); // 送球
    // PIDPosForwardAbs(200);

    PIDPosForwardAbs(0);                   // 第一颗球后退到中间
    PIDAngleRotateAbs(-78, 1.5, 0.05, 10); // 朝向第二颗球
    setIntakeSpeed(100);                   // 吸球
    PIDPosForwardAbs(350);                 // 前进
    voltageForward(40);                    // 吸到第二颗球
    this_thread::sleep_for(300);           // 吸到第二颗球
    setIntakeSpeed(30);                    // 吸球
    PIDPosForwardAbs(50);                  // 第二颗球后退到中间
    PIDAngleRotateAbs(120, 1.5, 0.05, 10); // 朝向goal
    setIntakeSpeed(-100);                  // 吐球
    posForwardAbsWithHeading(80, 150, 120);
    voltageForward(70); // 送球
    this_thread::sleep_for(450);
    // PIDPosForwardAbs(200);

    PIDPosForwardAbs(50);                   // 送完第二颗球后退到中间
    PIDAngleRotateAbs(-165, 1.5, 0.05, 10); // 朝向围板
    PIDPosForwardAbs(450);                  // 前进
    PIDAngleRotateAbs(-75, 1.5, 0.05, 10);  // 朝向第三颗球
    setIntakeSpeed(100);                    // 吸第三颗球
    voltageForward(70);                     // 吸到第三颗球
    this_thread::sleep_for(300);            // 吸到第三颗球
    // PIDPosForwardAbs(350);
    setIntakeSpeed(20);                   // 吸第三颗球
    PIDPosForwardAbs(50);                 // 回到中间
    PIDAngleRotateAbs(80, 1.5, 0.05, 10); // 朝向goal
    setIntakeSpeed(-100);                 // 吐球
    voltageForward(70);                   // 送第三颗球
    this_thread::sleep_for(700);
    PIDPosForwardAbs(0); // 回到中间
    // setPistonE(1);

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

void test() {
    MyTimer autotimer;
    autotimer.reset();

    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("Auton Time: %.1f                 ", autotimer.getTime());
}

// #ifdef ROBOT1
void runAuton(int auton_choose) {

    setAutonMode();
    autonFlipper(true);
    autonInit();

    if (auton_choose == 1)
        auton_near_1(); // 1+1+2 near
    else if (auton_choose == 2)
        auton_near_2(); //
    else if (auton_choose == 3)
        auton_far_1(); // 3+0+0 far
    else if (auton_choose == 4)
        auton_far_2(); //
}
// #endif