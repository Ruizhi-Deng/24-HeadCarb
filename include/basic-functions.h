#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

void delay(int msec);

// Basic movement
void setPistonLB(bool state);
void setPistonRB(bool state);
void setPistonF(bool state);
void moveIntaker(float percent);
void moveIntakerWithRPM(float RPM);

void moveLifter(float pct);

double IMUHeading();
double InitHeading();
void setIMUHeading(double deg);

void clearBrainScr();
void clearControllerScr();

// Output functions
// void setPiston_Deploy(bool);

#endif
