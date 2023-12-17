#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "robot-config.h"
#include "parameters.h"

using namespace vex;

/*  Index
//autonchecker
void autonFlipper(bool);
// Calculation
float abbs(float);
float deg2rad(float);
float rad2deg(float);
float sqrf(float);
int sign(float);
float deg2range(float);

// Output functions
void moveLeft(float);
void moveLeftVel(float);
void lockLeft(void);
void unlockLeft(void);
void moveRight(float);
void moveRightVel(float);
void lockRight(void);
void unlockRight(void);
void moveForward(float);
void moveForwardVel(float);
void moveClockwise(float);
void lockBase(void);
void unlockBase(void);
float getCrtVel();

// Input functions
float getLeftPos();
float getRightPos();
float getForwardPos();
void resetLeftPos();
void resetRightPos();
void resetForwardPos();

float getHeading();
void resetHeading();
void resetHeading(float);
float getPitch();

// Piston
void setPistonE1(bool);
void setPistonE2(bool);
void setPistonE(bool);
void setPistonHook(bool);

// Catapult
void setCataStatus(int, int=2);
int getCataStatus();
void catapult();

void setIntakeSpeed(float);
void intake();

void clearLowLiftStep();
void autoLowLift();
*/

/************************* Auton checker *************************/

/**
 * @brief Set the autonomous flipper status.
 *
 * This function sets the autonomous flipper status, which can be used to control specific behavior
 * in autonomous mode.
 *
 * @param _checker The status to set for the autonomous flipper.
 */
void autonFlipper(bool _checker);


/************************* Mathematics *************************/

/**
 * @brief Calculate the absolute value of a floating-point number.
 *
 * @param x The input value.
 * @return The absolute value of x.
 */
float abbs(float x);

/**
 * @brief Convert degrees to radians.
 *
 * @param deg The angle in degrees.
 * @return The angle in radians.
 */
float deg2rad(float deg);

/**
 * @brief Convert radians to degrees.
 *
 * @param rad The angle in radians.
 * @return The angle in degrees.
 */
float rad2deg(float rad);

/**
 * @brief Calculate the square of a floating-point number.
 *
 * @param x The input value.
 * @return The square of x.
 */
float sqrf(float x);

/**
 * @brief Determine the sign of a floating-point number.
 *
 * @param _input The input value.
 * @return 1 if _input is positive, -1 if _input is negative, and 0 if _input is zero.
 */
int sign(float _input);

/**
 * @brief Map an angle in degrees to the range [-180, 180].
 *
 * @param _degree The input angle in degrees.
 * @return The angle mapped to the range [-180, 180].
 */
float deg2range(float _degree);


/************************* Output functions *************************/

/**
 * @brief Move left base motors with a specified power.
 *
 * @param _input The power to apply to left base motors.
 */
void moveLeft(float _input);

/**
 * @brief Move left base motors with a specified velocity.
 *
 * @param _input The velocity to apply to left base motors.
 */
void moveLeftVel(float _input);

/**
 * @brief Lock left base motors.
 */
void lockLeft(void);

/**
 * @brief Unlock left base motors.
 */
void unlockLeft(void);

/**
 * @brief Move right base motors with a specified power.
 *
 * @param _input The power to apply to right base motors.
 */
void moveRight(float _input);

/**
 * @brief Move right base motors with a specified velocity.
 *
 * @param _input The velocity to apply to right base motors.
 */
void moveRightVel(float _input);

/**
 * @brief Lock right base motors.
 */
void lockRight(void);

/**
 * @brief Unlock right base motors.
 */
void unlockRight(void);

/**
 * @brief Move the base forward with a specified power.
 *
 * @param _input The power to apply to both sides of the base.
 */
void moveForward(float _input);

/**
 * @brief Move the base forward with a specified velocity.
 *
 * @param _input The velocity to apply to both sides of the base.
 */
void moveForwardVel(float _input);

/**
 * @brief Rotate the base clockwise with a specified power.
 *
 * @param _input The power to apply for clockwise rotation.
 */
void moveClockwise(float _input);

/**
 * @brief Lock both sides of the base.
 */
void lockBase(void);

/**
 * @brief Unlock both sides of the base.
 */
void unlockBase(void);


/************************* Input functions *************************/

/**
 * @brief Get the current velocity of the base.
 *
 * @return The current velocity of the base.
 */
float getCrtVel();

/**
 * @brief Get the position of left base motors.
 *
 * @return The position of left base motors in millimeters.
 */
float getLeftPos();

/**
 * @brief Get the position of right base motors.
 *
 * @return The position of right base motors in millimeters.
 */
float getRightPos();

/**
 * @brief Get the vertical position of the base.
 *
 * @return The vertical position of the base in millimeters.
 */
float getForwardPos();

/**
 * @brief Reset the position of left base motors.
 */
void resetLeftPos();

/**
 * @brief Reset the position of right base motors.
 */
void resetRightPos();

/**
 * @brief Reset the position of both sides of the base.
 */
void resetForwardPos();

/**
 * @brief Get the current heading angle of the robot.
 *
 * @return The current heading angle of the robot in degrees.
 */
float getHeading();

/**
 * @brief Reset the heading angle of the robot.
 */
void resetHeading();

/**
 * @brief Reset the heading angle of the robot with an offset.
 *
 * @param _offset The offset to apply to the heading angle reset.
 */
void resetHeading(float _offset);

/**
 * @brief Get the pitch angle of the robot.
 *
 * @return The pitch angle of the robot in degrees.
 */
float getPitch();


/**************************** Piston ****************************/
/**
 * @brief Set the state of the ExpansionRIGHT piston.
 *
 * @param _input The state to set for the ExpansionRIGHT piston.
 */
void setPistonE1(bool _input);

/**
 * @brief Set the state of the ExpansionLEFT piston.
 *
 * @param _input The state to set for the ExpansionLEFT piston.
 */
void setPistonE2(bool _input);

/**
 * @brief Set the state of both Expansion pistons.
 *
 * @param _input The state to set for both Expansion pistons.
 */
void setPistonE(bool _input);

/**
 * @brief Set the state of the Expansion piston.
 *
 * @param _input The state to set for the Expansion piston.
 */
void setPistonHook(bool _input);


/************************* Catapult *************************/

/**
 * @brief Set the catapult status and mode.
 *
 * @param status The status to set for the catapult.
 * @param mode The mode to set for the catapult.
 * 
 * CataStatus:  
 * 1 = ready to shoot    
 * 2 = shooting  
 * 5 = switch cata height
 * 4 = initial
 * 0 = pulling down to preshoot position
 */
void setCataStatus(int status, int mode = 2);

/**
 * @brief Get the current catapult status.
 *
 * @return The current catapult status.
 */
int getCataStatus();

/**
 * @brief Control the catapult mechanism.
 *
 * This function controls the catapult mechanism based on the `cataStatus` and `cataMode` variables.
 */
void catapult();


/*************************** Intake ***************************/

/**
 * @brief Set the speed of the intake mechanism.
 *
 * @param _input The speed to set for the intake mechanism.
 */
void setIntakeSpeed(float _input);

/**
 * @brief Run the intake mechanism continuously.
 */
void intake();


/************************* Lift **************************/

/**
 * @brief Clear the step variable for the low lift.
 */
void clearLowLiftStep();

/**
 * @brief Execute the low lift automation.
 */
void autoLowLift();



#endif
