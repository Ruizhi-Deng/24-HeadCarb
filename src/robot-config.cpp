#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);

#ifdef ROBOT1
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT15, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT13, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT2
  motor Motor_BaseLF = motor(PORT4, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT2, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT9, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT11, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT13, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT14, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT21, ratio36_1, true);

  inertial IMU = inertial(PORT20);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.D); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT3
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT18, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT15, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT14, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.H); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.C); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT4
  motor Motor_BaseLF = motor(PORT16, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT20, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT18, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT11, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT9, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT10, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT5
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT18, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT15, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT12, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT16);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.H); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.D); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.C); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT6
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT18, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT15, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT12, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.B); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.D); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.C); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT7
  motor Motor_BaseLF = motor(PORT1, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT2, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT3, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT4, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT5, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT6, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT8, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT7, ratio36_1, true);

  inertial IMU = inertial(PORT9);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.D); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.C); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT1, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT8
  motor Motor_BaseLF = motor(PORT3, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT4, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT5, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT6, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT7, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT13, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT11, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT14, ratio36_1, true);

  inertial IMU = inertial(PORT16);
  led PistonE1 = led(Brain.ThreeWirePort.B); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.A); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT9
  motor Motor_BaseLF = motor(PORT3, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT4, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT5, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT6, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT7, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT8, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT11, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT14, ratio36_1, true);

  inertial IMU = inertial(PORT16);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT10
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT19, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT20, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT15, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT6, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.B); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.A); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT11
  motor Motor_BaseLF = motor(PORT17, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT18, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT19, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT12, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT15, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

#ifdef ROBOT12
  motor Motor_BaseLF = motor(PORT9, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT2, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT4, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT11, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT14, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT18, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT10, ratio6_1, false);
  //motor Motor_Intake2 = motor(PORT21, ratio6_1, false);
  motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT21);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.C); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.B); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.E); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);
#endif

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
    wait(1000, msec);
    // setPistonE1(false);
    // setPistonE2(false);

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("IMU Calibrating");
    IMU.startCalibration();
    while (IMU.isCalibrating()) {
    }
    wait(1000, msec);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Done");
    wait(500, msec);
    Brain.Screen.clearScreen();

    // thread Intake(intake);
    // thread Catapult(catapult);
    // thread GPSPosition(MyGpsPos);
}

#ifdef ROBOT13
  motor Motor_BaseLF = motor(PORT7, ratio6_1, true);
  motor Motor_BaseLM = motor(PORT9, ratio6_1, true);
  motor Motor_BaseLB = motor(PORT8, ratio6_1, true);
  motor Motor_BaseRF = motor(PORT19, ratio6_1, false);
  motor Motor_BaseRM = motor(PORT18, ratio6_1, false);
  motor Motor_BaseRB = motor(PORT17, ratio6_1, false);
  motor Motor_Intake1 = motor(PORT15, ratio6_1, false);

  // // motor Motor_Cata1 = motor(PORT9, ratio36_1, true);

  inertial IMU = inertial(PORT12);
  led PistonE1 = led(Brain.ThreeWirePort.A); //Extension 1
  led PistonE2 = led(Brain.ThreeWirePort.H); //Extension 2
  led PistonHook = led(Brain.ThreeWirePort.D); //Hook
  limit limit1 = limit(Brain.ThreeWirePort.C); //Cata LIMIT
  //Vision Sensor 
  signature Vision18__SIG_1 = signature (1, 6605, 9541, 8073, 775, 1489, 1132, 3, 0);
  signature Vision18__SIG_2 = signature (2, 8191, 10083, 9137, -1527, -507, -1017, 3, 0);
  signature Vision18__SIG_3 = signature (3, 8239, 12205, 10222, -1081, -461, -772, 3, 0);
  signature Vision18__SIG_4 = signature (4, -3257, -2055, -2656, 8633, 11867, 10250, 3, 0);
  vision Vision18 = vision (PORT8, 65, Vision18__SIG_1, Vision18__SIG_2, Vision18__SIG_3);

#endif