#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "vex.h"

static int CRTCOLOR = 1; // 1 is red, 0 is blue

const float Movement_LowerLimit = 5;
const float Joystick_LowerDeadzone = 2;
volatile static float gpsPosX, gpsPosY, gpsHeading;
volatile static float vision_obj_x, vision_obj_y, vision_obj_height, vision_obj_width;

static const float PI = 3.1416;
//0: red, 1:blue, 2:both
static volatile int target_blue = 0;

#ifdef ROBOT1
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT2
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT3
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1800.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT4
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT5
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT6
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT7
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT8
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT9
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT10
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT11
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#ifdef ROBOT12
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif



#ifdef ROBOT13
  static const float LEFT_MAX_SPD = 82;
  static const float LEFT_DELTA_SPD = 100-LEFT_MAX_SPD;
  static const float WHEEL_DIAMETER = 4;
  static const float CHASSIS_GEAR_RATIO = 0.5;
  static const float CHASSISRADIUS = 156;
  static const float kGyro = 1830.0 / 1800.0;
  static const float INTAKEMIDPOS = 50;
#endif

#endif