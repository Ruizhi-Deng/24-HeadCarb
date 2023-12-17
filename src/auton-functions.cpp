#include "auton-functions.h"

#include "PID.h"
#include "basic-functions.h"
#include "iostream"
#include "my-timer.h"
#include "parameters.h"
#include "vex.h"
using namespace vex;
using namespace std;

/**
 * Moves both the left and right motors forward at the specified power level.
 *
 * @param _power The power level at which to move the motors forward.
 */
void voltageForward(float _power) {
    moveLeft(_power);
    moveRight(_power);
    this_thread::sleep_for(5);
}

/**
 * Moves the base forward at a specified power for a specified duration.
 * This function does not stop the base when finishing.
 *
 * @param _power The power at which to move the base forward.
 * @param _duration The duration in milliseconds for which to move the base
 * forward.
 */
void timerForward(float _power, int _duration) {
    moveForward(_power);
    this_thread::sleep_for(_duration);
    unlockBase();
}

/**
 * Moves the robot forward with a specified power and duration while maintaining
 * a target heading.
 *
 * @param _power The power at which the robot should move forward.
 * @param _duration The duration for which the robot should move forward.
 * @param _targetHeading The target heading that the robot should maintain while
 * moving forward.
 */
void timerForwardWithHeading(float _power, int _duration, float _targetHeading) {
    auto myTimer = MyTimer();
    while (myTimer.getTime() < _duration) {
        float headingError = _targetHeading - getHeading();
        float powerTurn = headingError * 2.0; // kp = 2.0
        if (fabs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        moveLeft(_power + powerTurn);
        moveRight(_power - powerTurn);
    }
    unlockBase();
}

/**
 * Moves the base forward with a specified power for a given displacement.
 * The base does not stop when reaching the target displacement.
 * The direction of movement is determined by the sign of the power parameter.
 *
 * @param _power The power at which the base should move forward.
 * @param _target The target displacement for the base movement.
 */
void posForwardRel(float _power, float _target) {
    float startPos = getForwardPos();
    moveForward(_power);
    while (fabs(getForwardPos() - startPos) < fabs(_target)) {
    } // wait until reaching target
    unlockBase();
}

/**
 * Moves the base forward with a specified power to reach a target position.
 * The base does not stop when reaching the target position.
 * The direction of movement is determined by the sign of (_target - input).
 *
 * @param _power The power at which the base should move forward.
 * @param _target The target position to reach.
 */
void posForwardAbs(float _power, float _target) {
    float targetRel = _target - getForwardPos();
    float power = sign(targetRel) * fabs(_power);
    posForwardRel(power, targetRel);
}

/**
 * Moves the robot forward relative to its current position and heading.
 *
 * @param _power The power at which the robot should move.
 * @param _targetPos The target position relative to the current position.
 * @param _targetHeading The target heading relative to the current heading.
 */
void posForwardRelWithHeading(float _power, float _targetPos, float _targetHeading) {
    // move forward with _power for _targetPos displacement while sticking to
    // _targetHeading angle does not stop base when finishing use the sign of
    // _power to determine direction
    float startPos = getForwardPos();
    while (fabs(getForwardPos() - startPos) < fabs(_targetPos)) {
        float headingError = deg2range(_targetHeading - getHeading());
        float powerTurn = headingError * 10.0; // kp = 3.0
        if (fabs(powerTurn) > 100)
            powerTurn = sign(powerTurn) * 100; // PLimit = 100

        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("TAR: %.2f|CURR: %.2f|ERR: %.2f|PTURN: %.2f", _targetHeading,
                           getHeading(), headingError, powerTurn);

        moveLeft(_power + powerTurn);
        moveRight(_power - powerTurn);

        this_thread::sleep_for(50);
        Brain.Screen.clearScreen();
    }
    unlockBase();
}

/**
 * Moves the robot forward to a specified absolute position while maintaining a target heading.
 *
 * @param _power The power level for the movement.
 * @param _targetPos The target position to move to.
 * @param _targetHeading The target heading to maintain during the movement.
 */
void posForwardAbsWithHeading(float _power, float _targetPos, float _targetHeading) {
    // move forward with _power to _target position while sticking to
    // _targetHeading angle does not stop base when finishing use the sign of
    // (_target - input) to determine direction
    float targetPosRel = _targetPos - getForwardPos();
    float power = sign(targetPosRel) * fabs(_power);
    posForwardRelWithHeading(power, targetPosRel, _targetHeading);
}

/**
 * Moves the base forward to the specified target position using PID control.
 * Stops the base when the target position is reached.
 *
 * @param _target The target position to move forward to.
 */
void PIDPosForwardAbs(float _target) {
    // move forward to _target position
    // stops base when finishing
    auto pid = PID();
    pid.setCoefficient(0.5, 0.03, 5);
    pid.setTarget(_target);
    pid.setIMax(30);
    pid.setIRange(10);
    pid.setErrorTolerance(5);
    pid.setDTolerance(20);
    pid.setJumpTime(50);
    while (!pid.targetArrived()) { //} && myTimer.getTime() < 1000 +
                                   // abbs(target * 10)) {
        pid.update(getForwardPos());
        moveForward(pid.getOutput());
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Forward Position: %.1f                           ", getForwardPos());
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * Moves the robot forward to a relative target position using PID control.
 *
 * @param _target The relative target position to move to.
 */
void PIDPosForwardRel(float _target) {
    // move forward for _target displacement
    // stops base when finishing
    PIDPosForwardAbs(getForwardPos() + _target);
}

/**
 * @brief Performs a PID control to move the robot to a target position with a curved path.
 *
 * @param left_target The target position for the left side of the robot.
 * @param right_target The target position for the right side of the robot.
 * @param tolerance The acceptable error tolerance for reaching the target position.
 */
void PIDPosCurveRel(float left_target, float right_target, float tolerance) {
    // move curved to _target position
    // stops base when finishing

    float _target = (left_target + right_target) / 2;
    float ratio = left_target / right_target;
    auto pid = PID();
    float k = 1;
    pid.setCoefficient(1.05, 0.05, 1.5);
    pid.setTarget(_target);
    pid.setIMax(30);
    pid.setIRange(20);
    pid.setErrorTolerance(tolerance);
    pid.setDTolerance(5);
    pid.setJumpTime(50);
    while (!pid.targetArrived()) { //} && myTimer.getTime() < 1000 +
                                   // abbs(target * 10)) {
        float leftPos_err = (getForwardPos() / _target) * left_target - getLeftPos();
        float rightPos_err = (getForwardPos() / _target) * right_target - getRightPos();
        pid.update(getForwardPos());
        float PIDoutput = pid.getOutput();
        if (fabs(PIDoutput) > 90)
            PIDoutput = sign(PIDoutput) * 90;
        if (ratio > 1) {
            moveLeft(PIDoutput + k * leftPos_err);
            moveRight(PIDoutput / ratio + k * rightPos_err);
        } else {
            moveLeft(pid.getOutput() * ratio + k * leftPos_err);
            moveRight(pid.getOutput() + k * rightPos_err);
        }
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Forward Position: %.1f                           ", getForwardPos());
        this_thread::sleep_for(5);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * Calculates the PID control for position curve with absolute target values.
 *
 * @param left_target The target value for the left side.
 * @param right_target The target value for the right side.
 * @param tolerance The tolerance for error in position.
 */
void PIDPosCurveAbs(float left_target, float right_target, float tolerance) {
    PIDPosCurveRel(getLeftPos() + left_target, getRightPos() + right_target, tolerance);
}

/**
 * Softly starts a timer-controlled forward movement with gradually increasing power.
 *
 * @param _powerInit The initial power of the movement.
 * @param _powerFinal The final power of the movement.
 * @param _duration The duration of the movement in milliseconds.
 */
void softStartTimerForward(float _powerInit, float _powerFinal, int _duration) {
    // move forward with power gradually increase from _powerInit to _powerFinal
    // within _duration msec does not stop base when finishing
    auto myTimer = MyTimer();
    float step = (_powerFinal - _powerInit) / _duration;
    while (myTimer.getTime() < _duration) {
        moveForward(_powerInit + myTimer.getTime() * step);
        this_thread::sleep_for(5);
    }
    unlockBase();
}

/**
 * @brief Rotates the timer for a specified duration with a given power.
 *
 * @param _power The power at which to rotate the timer.
 * @param _duration The duration for which to rotate the timer.
 */
void timerRotate(float _power, int _duration) {
    // rotate clockwise with _power for _duration msec
    // does not stop base when finishing
    moveClockwise(_power);
    this_thread::sleep_for(_duration);
    resetForwardPos();
    unlockBase();
}

/**
 * Rotate the angle relative to the current position.
 *
 * @param _power The power of the rotation.
 * @param _target The target angle to rotate to.
 */
void angleRotateRel(float _power, float _target) {
    // rotate clockwise with _power for _target angle
    // does not stop base when finishing
    float startAngle = getHeading();
    float power = sign(_target) * fabs(_power);
    moveClockwise(power);
    while (fabs(getHeading() - startAngle) < fabs(_target)) {
    }
    resetForwardPos();
    unlockBase();
}

/**
 * Rotate the angle of the robot to an absolute target angle.
 *
 * @param _power The power at which the robot should rotate.
 * @param _target The target angle to rotate to.
 */
void angleRotateAbs(float _power, float _target) {
    // rotate clockwise with _power to _target angle
    // does not stop base when finishing
    angleRotateRel(_power, _target - getHeading());
}

/**
 * Rotates the base clockwise with a specified power for a relative target angle.\
 * Use PID control
 * Stops the base when the rotation is complete.
 *
 * @param _target The relative target angle to rotate.
 */
void PIDAngleRotateRel(float _target) {
    // rotate clockwise with _power for _target angle
    // stops base when finishing
    PIDAngleRotateAbs(getHeading() + _target);
}

/**
 * @brief Rotate the robot to an absolute angle using PID control.
 *
 * @param _target The target angle to rotate to.
 * @param kp The proportional gain for PID control.
 * @param ki The integral gain for PID control.
 * @param kd The derivative gain for PID control.
 * @param tolerance The acceptable error tolerance for reaching the target angle.
 */
void PIDAngleRotateAbs(float _target, float kp, float ki, float kd, float tolerance) {
    // rotate clockwise with _power to _target angle
    // stops base when finishing

    // auto myTimer = MyTimer();
    auto pid = PID();
    // pid.setCoefficient(0.9, 0.1, 3);
    pid.setCoefficient(kp, ki, kd);
    pid.setTarget(_target);
    pid.setIMax(30);
    pid.setIRange(15);
    pid.setErrorTolerance(tolerance);
    pid.setDTolerance(5);
    pid.setJumpTime(50);
    pid.setType("turn");
    while (!pid.targetArrived()) { // &&  myTimer.getTime() < 1000 +
                                   // abbs(target * 3)) {
        pid.update(getHeading());
        moveClockwise(pid.getOutput());
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Heading: %.2f            ", getHeading());
        this_thread::sleep_for(5);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * This function gradually increases the power from _powerInit to _powerFinal over a specified
 * duration.
 *
 * @param _powerInit The initial power value.
 * @param _powerFinal The final power value.
 * @param _duration The duration over which the power is gradually increased.
 */
void softStartTimerRotate(float _powerInit, float _powerFinal, int _duration) {
    // rotate clockwise with power gradually increase from _powerInit to
    // _powerFinal within _duration msec does not stop base when finishing
    auto myTimer = MyTimer();
    float step = (_powerFinal - _powerInit) / _duration;
    while (myTimer.getTime() < _duration) {
        moveClockwise(_powerInit + myTimer.getTime() * step);
    }
    resetForwardPos();
    unlockBase();
}

/**
 * Waits for a specified amount of time.
 *
 * @param _waittime The time to wait in seconds.
 */
void timerWait(float _waittime) { this_thread::sleep_for(_waittime); }

static bool autonMode = false;
void clearAutonMode(void) { autonMode = false; }
void setAutonMode(void) { autonMode = true; }
