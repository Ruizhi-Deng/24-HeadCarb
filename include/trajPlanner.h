#ifndef _TRAJPLANNER_H_
#define _TRAJPLANNER_H_
#include "geometry.h"
#include <vector>
using namespace std;
#include <iomanip>
#include <iostream>

class Traj {
  private:
  public:
    vector<Point> wayPoint;
    // double *c[4];
    Traj() {
    }
    Traj(vector<Point> &t) {
        swap(wayPoint, t);
    }
    Traj(int n) {
        // for(int i = 0; i <= 3; ++i)
        //     c[i] = new double[n];
    } 
    void push(Point p) {
        wayPoint.push_back(p);
    }
    Point getGoalPoint(double x, double y) {
        int nearest = 0;
        double nearestLength = 10000000;
        int tar = wayPoint.size() - 1;
        Point cur = Point(x, y);
        for (int i = 0; i < wayPoint.size(); ++i) {
            double len = (wayPoint[i] - cur).mod();
            if (len < nearestLength) {
                nearestLength = len;
                nearest = i;
            }
        }
        for (int i = nearest; i < wayPoint.size(); ++i) {
            double len = (wayPoint[i] - cur).mod();
            if (len > 2) {
                tar = i;
                break;
            }
        }
        return wayPoint[tar];
    }
    Point getTarget() {
        return wayPoint[wayPoint.size() - 1];
    }
    ~Traj() {
        // delete[] c;
    }
};

class TrajPlanner {
  private:
    vector<Pose> wayPoint;

  public:
    void addWayPoint(Pose p) {
        wayPoint.push_back(p);
    }
    void addWayPoint(double x, double y, double dir) {
        wayPoint.push_back(Pose(x, y, dir));
    }
    Traj generateBezierTraj(int n, bool report = false) {
        if (n < 100) n = 100;
        double(*c)[4] = new double[wayPoint.size() + 1][4];
        Traj BezierTraj;
        for (int i = 0; i < wayPoint.size() - 1; ++i) {
            Point p1(wayPoint[i]._x, wayPoint[i]._y);
            Point p4(wayPoint[i + 1]._x, wayPoint[i + 1]._y);
            double dir1 = 90 - wayPoint[i]._dir;
            double dir4 = 90 - wayPoint[i]._dir;
            double len = (p4 - p1).mod() / 2;
            Point p2 = p1 + Vector(len, 0).rotateTrans(dir1);
            Point p3 = p4 + Vector(len, 0).rotateTrans(dir4);
            // P(t) = (1-t)^3 * p1 + 3*t*(1-t)^2 * p2 + 3t^2*(1-t) * p3 + t^3 * p4
            for (int i = 1; i <= n; ++i) {
                double t = double((i - 1)) / double((n - 1));
                double px = (1 - t) * (1 - t) * (1 - t) * p1._x +
                            3 * t * (1 - t) * (1 - t) * p2._x +
                            3 * t * t * (1 - t) * p3._x +
                            t * t * t * p4._x;
                double py = (1 - t) * (1 - t) * (1 - t) * p1._y +
                            3 * t * (1 - t) * (1 - t) * p2._y +
                            3 * t * t * (1 - t) * p3._y +
                            t * t * t * p4._y;
                BezierTraj.push(Point(px, py));
            }
        }
        if (report) {
            cout << "------ Traj. Points: ------" << endl;
            for (int i = 0; i < BezierTraj.wayPoint.size(); ++i) {
                cout << fixed << setprecision(3);
                cout << "\t(" << BezierTraj.wayPoint[i]._x << "," << BezierTraj.wayPoint[i]._y << ")" << endl;
            }
        }

        return BezierTraj;
    }
};

#endif