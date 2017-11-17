#include <iostream>
#include <fstream>

#include <functional>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "helpers.h"

using namespace std;


Pose::Pose() {
    x = 0;
    y = 0;
    theta = 0;
}

Pose::Pose(double _x, double _y, double _theta):
    x(_x),y(_y),theta(_theta) {}

Pose::Pose(const Pose& _p):
    x(_p.x),y(_p.y),theta(_p.theta) {}

double Pose::heading_diff_to_pose(Pose p) {
    Eigen::Vector2d heading = Eigen::Vector2d(cos(theta), sin(theta));
    Eigen::Vector2d go_to_p = Eigen::Vector2d(x - p.x, y - p.y).normalized();

    double cross_prod_sign = heading.x()*go_to_p.y() - heading.y()*go_to_p.x();
    if (cross_prod_sign < 0) {
        return -acos(heading.dot(go_to_p));
    }

    return acos(heading.dot(go_to_p));
}

double Pose::update_heading(double angle) {
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

LineSegment::LineSegment(){
    a = Pose();
    b = Pose();
}

LineSegment::LineSegment(Pose _a, Pose _b):
    a(_a), b(_b) {}

double LineSegment::dist_to_point_sq(Pose p) {
    double A, B, C, D, dot, len_sq, param, xx, yy;
    A = p.x - a.x;
    B = p.y - a.y;
    C = b.x - a.x;
    D = b.y - a.y;

    dot = (A * C) + (B * D);
    len_sq = C*C + D*D;

    param = -1;
    if (len_sq > 0.001) { // basically zero length with float arithmetic
        param = dot / len_sq;
    }

    if (param < 0) {
        xx = a.x;
        yy = a.y;
    }
    else if (param > 1) {
        xx = b.x;
        yy = b.y;
    }
    else {
        xx = a.x + param*C;
        yy = a.y + param*D;
    }
    double dx, dy;
    dx = p.x - xx;
    dy = p.y - yy;

    return dx*dx + dy*dy;
}

std::ostream& operator<<(std::ostream& os, const Pose& p) {
    return os << "(" << p.x << ", " << p.y << ", " << (180*p.theta/PI) << ")";
};

std::ostream& operator<<(std::ostream& os, const LineSegment& l) {
    return os << "{" << l.a << "->" << l.b << "}";
}
