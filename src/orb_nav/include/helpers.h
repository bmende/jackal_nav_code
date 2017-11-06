#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using namespace std;

const double PI = 3.141592653589793238463;

struct Pose {
    double x, y, theta;
    double dist(Pose p) {
        return sqrt((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y));
    }

    double heading_diff_to_pose(Pose p) {
        Eigen::Vector2d heading = Eigen::Vector2d(cos(theta), sin(theta));
        Eigen::Vector2d go_to_p = Eigen::Vector2d(p.x - x, p.y - y).normalized();

        double cross_prod_sign = heading.x()*go_to_p.y() - heading.y()*go_to_p.x();
        if (cross_prod_sign < 0) {
            return -acos(heading.dot(go_to_p));
        }

        return acos(heading.dot(go_to_p));
    }

    double update_heading(double angle) {
        Eigen::Vector2d heading = Eigen::Vector2d(cos(theta), sin(theta));
        Eigen::Vector2d update = Eigen::Vector2d(cos(angle), sin(angle));

        cout << "theta: " << theta << " update: " << angle;
        //theta = acos(heading.dot(update));


        theta += angle;
        if (theta < -2*PI) {
            theta += 2*PI;
        }
        if (theta >= 2*PI) {
            theta -= 2*PI;
        }
        cout << " new theta: " << theta << endl;
    }

    friend std::ostream& operator<<(std::ostream&, const Pose&);
};

std::ostream& operator<<(std::ostream& os, const Pose& p) {
    return os << "(" << p.x << ", " << p.y << ", " << p.theta << ")";
};

#endif // HELPERS_H
