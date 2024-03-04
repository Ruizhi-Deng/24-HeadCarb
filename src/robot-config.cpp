#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors

motor Motor_BaseR1 = motor(PORT4, ratio6_1, true);
motor Motor_BaseR2 = motor(PORT5, ratio6_1, false);
motor Motor_BaseR3 = motor(PORT7, ratio6_1, true);
motor Motor_BaseR4 = motor(PORT8, ratio6_1, false);
motor Motor_BaseR5 = motor(PORT9, ratio6_1, true);

motor Motor_BaseL1 = motor(PORT14, ratio6_1, false);
motor Motor_BaseL2 = motor(PORT15, ratio6_1, true);
motor Motor_BaseL3 = motor(PORT17, ratio6_1, false);
motor Motor_BaseL4 = motor(PORT18, ratio6_1, true);
motor Motor_BaseL5 = motor(PORT19, ratio6_1, false);

motor Motor_Lifter1 = motor(PORT13, ratio18_1, true);
motor Motor_Lifter2 = motor(PORT3, ratio18_1, false);

controller Controller = controller(primary);

inertial Inertial = inertial(PORT1);

motor Motor_Intaker = motor(PORT20, ratio18_1, false);

digital_out Piston_FO = digital_out(Brain.ThreeWirePort.G);
digital_out Piston_FI = digital_out(Brain.ThreeWirePort.H);
digital_out Piston_BLO = digital_out(Brain.ThreeWirePort.B);
digital_out Piston_BLI = digital_out(Brain.ThreeWirePort.A);
digital_out Piston_BRO = digital_out(Brain.ThreeWirePort.E);
digital_out Piston_BRI = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
}
