#ifndef AUTON_FUNCTIONS_H_
#define AUTON_FUNCTIONS_H_
#include "basic-functions.h"
#include "PID.h"
#include "positionNew.h"
#include "my-timer.h"
#include "trajPlanner.h"
#include "chassis.h"
#include <iostream>

void aimAt(double _x, double _y, double _offset);
void turnTo(double _tarAng, double _maxSpeed);
void timerForward(float _power, int _duration);
void timerForwardWithHeading(float _power, int _duration, double _targetHeading);
void moveWhileAiming(float _power, double _tarX, double _tarY, double _offset);
void timerForwardWhileAiming(float _power, int _duration, double _tarX, double _tarY);
void posForwardRel(float _power, float _target);
void posForwardRelWithHeading(float _power, float _targetPos, float _targetHeading);
void posForwardRelWhileAiming(float _power, float _targetPos, double _tarX, double _tarY);
void PIDPosForwardRel(float _target, bool _clawF, bool _clawB);
void softStartTimerForward(float _powerInit, float _powerFinal, int _duration);
void timerRotate(float _power, int _duration);
void angleRotateRel(float _power, float _target);
void angleRotateAbs(float _power, float _target);
void PIDAngleRotateRel(float _target);
void PIDAngleRotateAbs(float _target);
void softStartTimerRotate(float _powerInit, float _powerFinal, int _duration);
void trajMove(Traj traj, double _power, directionType direction = directionType ::fwd);

void timerForwardWithHeading(float _power, int _duration, double _targetHeading);
void timerMove(Vector vel, int msec);

void aimAt(double _x, double _y, double _offset = 0);

void turnTo(double _tarAng, double _maxSpeed = 50);

#endif