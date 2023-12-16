/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                    \
  for (int iterator = 0; iterator < iterations; iterator++)

//#define ROBOT1  // 54189K
//#define ROBOT2  // 54189J
//#define ROBOT3  // 12377X
//#define ROBOT4  // 12377Y
//#define ROBOT5  // 12377Z
//#define ROBOT6  // 96331A
//#define ROBOT7  // 96331B华二
//#define ROBOT8  // 38825A交二
//#define ROBOT9  // 38825B新交二
//#define ROBOT10  // 98731A
//#define ROBOT11  // 98731B
//#define ROBOT12
#define ROBOT13 //SJTU3 overunder

#define BUTTONREGULAR
//#define BUTTONREVERSE
