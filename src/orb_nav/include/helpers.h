#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <fstream>

#include <functional>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

using namespace std;

const double PI = 3.141592653589793238463;

typedef pair<int, int> int_pair;
struct int_pair_hash {
    size_t operator()(int_pair p) const noexcept {
        return size_t(p.first) << 32 | p.second;
    }
};

class Pose {
public:
    double x, y, theta;


    Pose() {
        x = 0;
        y = 0;
        theta = 0;
    }

    Pose(double _x, double _y, double _theta):
        x(_x),y(_y),theta(_theta) {}

    Pose(const Pose& _p):
        x(_p.x),y(_p.y),theta(_p.theta) {}

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

class LineSegment {
    Pose a, b;

public:

    LineSegment(){
        a = Pose();
        b = Pose();
    }

    LineSegment(Pose _a, Pose _b):
        a(_a), b(_b) {}

    double dist_to_point_sq(Pose p) {
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

    friend std::ostream& operator<<(std::ostream&, const LineSegment&);

};

std::ostream& operator<<(std::ostream& os, const Pose& p) {
    return os << "(" << p.x << ", " << p.y << ", " << p.theta << ")";
};

std::ostream& operator<<(std::ostream& os, const LineSegment& l) {
    return os << "{" << l.a << "->" << l.b << "}";
}

#endif // HELPERS_H
