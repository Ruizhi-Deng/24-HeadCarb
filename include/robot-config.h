#ifndef ROBOT_CONFIG_H_
#define ROBOT_CONFIG_H_
#include "vex.h"

using namespace vex;

const auto ENCODERPORT = PORT21;

extern brain Brain;

// VEXcode device constructors

extern motor Motor_BaseL1;
extern motor Motor_BaseL2;
extern motor Motor_BaseL3;
extern motor Motor_BaseL4;
extern motor Motor_BaseL5;

extern motor Motor_BaseR1;
extern motor Motor_BaseR2;
extern motor Motor_BaseR3;
extern motor Motor_BaseR4;
extern motor Motor_BaseR5;

extern motor Motor_Lifter1;
extern motor Motor_Lifter2;

extern controller Controller;

extern inertial Inertial;

extern motor Motor_Intaker;

extern digital_out Piston_FO;
extern digital_out Piston_FI;
extern digital_out Piston_BLO;
extern digital_out Piston_BLI;
extern digital_out Piston_BRO;
extern digital_out Piston_BRI;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

#endif