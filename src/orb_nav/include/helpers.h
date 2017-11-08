#pragma once
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


    Pose();

    Pose(double _x, double _y, double _theta);

    Pose(const Pose& _p);

    inline double dist(Pose p) { return sqrt((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y)); }

    double heading_diff_to_pose(Pose p);

    double update_heading(double angle);

    friend std::ostream& operator<<(std::ostream&, const Pose&);
};

class LineSegment {
    Pose a, b;

public:

    LineSegment();

    LineSegment(Pose _a, Pose _b);

    double dist_to_point_sq(Pose p);

    friend std::ostream& operator<<(std::ostream&, const LineSegment&);

};

#endif // HELPERS_H
