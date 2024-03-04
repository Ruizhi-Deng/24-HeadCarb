#ifndef POSITIONNEW_H_
#define POSITIONNEW_H_
#include "parameters.h"
#include "geometry.h"
#include "robot-config.h"
#include "my-timer.h"
#include <mutex>
using namespace vex;
class Position{
private:
    double IMUHeading;
    double sampleTime, lastTime;

    
    double Xfix, Yfix;
    double initX, initY, initHeading;
    double initXOffset, initYOffset;

    double globalYSpeed, globalXSpeed;
    
    double globalY, globalX;
    double lastGlobalX, lastGlobalY, lastIMUHeading;
    double lastSensorX, lastSensorY, lastSensorHeading;
    mutex locker;
    uint8_t data[32];
    MyTimer Timer;

    Position();
    ~Position();
    void getData();
    void updateGlobalSpeed();
    void updateGlobalPos();

public:
    double sensorX, sensorY, sensorHeading;
    static Position *getInstance(){
        static Position *p = NULL;
        if (p == NULL){
            p = new Position();
        }
        return p;
    }
    static void deleteInstance(){
        Position *p = Position::getInstance();
        if(p != NULL){
            delete p;
            p = NULL;
        }
    }
    void updatePos();
    Point getPos() const;
    double getXSpeed() const;
    double getYSpeed() const;
    double getIMUHeading() const;
    double getInitHeading() const;
    double getSensorHeading() const;
    void resetYPosition();
    void resetXPosition();
    void setGlobalPosition(double _x, double _y);
    void setHeading(double head);
};

void updatePosition();
#endif