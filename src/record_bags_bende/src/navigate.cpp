#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ctime>
#include <fstream>
#include <deque>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
//#include "popt_pp.h"

using namespace std;
using namespace cv;

ros::Publisher vel_pub;

double forward_vel = 0., rot_vel = 0.;
double trans_accel = 0.025; // forward acceleration
double trans_decel = 0.1; // forward deceleration
double rot_accel = 0.05; // rotational acceleration
float max_forward_vel = 1.0; // maximum forward velocity
double max_rot_vel = 0.6; // maximum rotational velocity

bool triangle_pressed = false;

// declare clearances in front and side of the robot
double clear_front = 0.24 + 0.8;
double clear_side = 0.3;

deque< int > commands;
int last_dir = 0;

const int INF = 1e9;

struct Pose
{
    double x, y, theta;
    double dist(Pose p) {
        return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
    }
};

struct LineSegment
{
    float x1, y1, x2, y2;
    float getAngle(LineSegment l) {
        float m1 = atan2(y2-y1,x2-x1);
        float m2 = atan2(l.y2-l.y1,l.x2-l.x1);
        return m1 - m2;
    }
    float heading() {
        return atan2(y2-y1,x2-x1);
    }
};

Pose jackal_pos = {0,0,0};
Pose last_jackal_pos = {0,0,0};
Pose current_waypoint;
bool reached_waypoint = false;
deque< Pose > path;
int pose_update_counter = 0;
int rot_frames = 0;


pair< double, double > goToWayPoint(Pose wayPoint, double front) {
    pair< double, double > ret_vel;
    double dist = wayPoint.dist(jackal_pos);
    if (dist < 0.5) {
        reached_waypoint = true;
        cout << "reached waypoint " << wayPoint.x << ", " << wayPoint.y << " : " << jackal_pos.x << ", "<< jackal_pos.y << endl;
        ret_vel = make_pair(0.,0.);
    } else if (rot_frames != 0) {
        if (rot_frames < 0) {
            cout << "ROTATING LEFT!!!" << endl;
            ret_vel.second = max_rot_vel * 0.5;
            rot_frames++;
        } else {
            cout << "ROTATING RIGHT!!!" << endl;
            ret_vel.second = -max_rot_vel * 0.5;
            rot_frames--;
        }
        ret_vel.first = 0.;//max_forward_vel * max(0.4, front);
    } else {
        ret_vel.first = max_forward_vel * max(0.4, front);
        ret_vel.second = 0.;
    }
    cout << "Distance to WP: " << dist << endl;
    return ret_vel;
}

pair< double, double > autoNavigateMode(double front) {
    pair< double, double > ret_vel;
    ret_vel = make_pair(0.,0.);
    if (path.size() == 0 && reached_waypoint) {
        cout << "REACHED ALL WAYPOINTS" << endl;
        return ret_vel;
    }
    if (reached_waypoint) {
        cout << "REACHED WAYPOINT! *********************************" << endl;
        current_waypoint = path.front();
        path.pop_front();
        reached_waypoint = false;
    }
    if (!reached_waypoint) {
        ret_vel = goToWayPoint(current_waypoint, front);
    }
    cout << "Current waypoint: " << current_waypoint.x << ", " << current_waypoint.y << endl;
    return ret_vel;
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
    // read joystick input
    int R2 = msg->buttons[9];
    int R1 = msg->buttons[11];
    int X = msg->buttons[14];
    int O = msg->buttons[13];
    int triangle = msg->buttons[12];
    double side = msg->axes[0];
    double front = msg->axes[1];
    double desired_forward_vel, desired_rot_vel;
    pair< double, double > desired_vel;
    // run the different modes

    if (triangle) {
        desired_vel = autoNavigateMode(front); // navigation doesn't work yet
        triangle_pressed = true;
    } else {
        triangle_pressed = false;
        return;
    }
    // accelerate or decelerate accordingly
    desired_forward_vel = desired_vel.first;
    desired_rot_vel = desired_vel.second;
    if (desired_forward_vel < forward_vel) {
        forward_vel = max(desired_forward_vel, forward_vel - trans_decel);
    } else {
        forward_vel = min(desired_forward_vel, forward_vel + trans_accel);
    }
    if (desired_rot_vel < rot_vel) {
        rot_vel = max(desired_rot_vel, rot_vel - rot_accel);
    } else {
        rot_vel = min(desired_rot_vel, rot_vel + rot_accel);
    }

    cout << "Forward Vel: " << forward_vel << ", Rot Vel: " << rot_vel << endl;

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = forward_vel;
    vel_msg.angular.z = rot_vel;
    vel_pub.publish(vel_msg);
}

void getCurrentPose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    jackal_pos.x = msg->pose.position.x;
    jackal_pos.y = msg->pose.position.y;

    // get yaw! first get quaternion, then turn to yaw
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    jackal_pos.theta = yaw;
    pose_update_counter++;
    //cout << "Current: " << jackal_pos.x << ", " << jackal_pos.y << " Prev: " << last_jackal_pos.x << ", " << last_jackal_pos.y << endl;

    LineSegment heading_line = {last_jackal_pos.x,last_jackal_pos.y,jackal_pos.x,jackal_pos.y};
    LineSegment waypoint_line = {jackal_pos.x,jackal_pos.y,current_waypoint.x,current_waypoint.y};
    //cout << "Heading: " << (heading_line.heading() * 180. / 3.14) << endl;
    double ang_diff = heading_line.getAngle(waypoint_line);
    //cout << "Ang diff: " << (ang_diff * 180. / 3.14)  << endl;
    //cout << "Rot frames: " << rot_frames << endl;

    if (!triangle_pressed)
        cout << "Current position: " << jackal_pos.x << ", " << jackal_pos.y << ", " << (jackal_pos.theta*180/3.14) << ". AngleDiff: " << (ang_diff*180/3.14) <<endl;


    if (pose_update_counter > 20) {
        if (last_jackal_pos.dist(jackal_pos) > 0.1) {
            if (abs(ang_diff * 180 / 3.14) > 10) {
                double cmd_rate = 8.;
                rot_frames = ang_diff * cmd_rate / (max_rot_vel * 0.5);
            } else {
                rot_frames = 0;
            }
            last_jackal_pos = jackal_pos;
        }
        pose_update_counter = 0;
    }
}

void read_waypoints(char* filename) {
    ifstream f(filename);
    if (f.is_open()) {
        int n;
        Pose waypoint;
        float x, y;
        waypoint.theta = 0.;
        f >> n;
        for (int i = 0; i < n; i++) {
            f >> x;
            f >> y;
            waypoint.x = x;
            waypoint.y = y;
            cout << "Read (" << waypoint.x << "," << waypoint.y << ")" << endl;
            path.push_back(waypoint);
        }
        current_waypoint = path.front();
        path.pop_front();
        cout << "Read waypoints!" << endl;
    } else {
        cout << "Cannot open file!" << endl;
    }
}

void test_waypoints() {

    Pose w1, w2, w3, w4, w5;
    w1.x = 0.1; w1.y = 0.1; w1.theta = 0.;
    w2.x = 6.; w2.y = 0.1; w2.theta = 0.;
    w3.x = 12.; w3.y = 0.1; w3.theta = 0.;
    w4.x = 28.5; w4.y = 0.1; w4.theta = 0.;
    w5.x = 28.5; w5.y = -10.1; w5.theta = 0.;
    path.push_back(w2);
    path.push_back(w3);
    path.push_back(w4);
    path.push_back(w5);
    current_waypoint = w1;

    cout << "test waypoints added\n";

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "orb_naive_navigation");
    ros::NodeHandle nh;

    // static struct poptOption options[] = {
    //     { "max-forward-vel",'f',POPT_ARG_FLOAT,&max_forward_vel,0,"Max forward velocity","NUM" },
    //     { "laser-thresh",'l',POPT_ARG_INT,&laser_pt_thresh,0,"Threshold for obstacle scan","NUM" },
    //     { "forward-clearance",'c',POPT_ARG_FLOAT,&clear_front,0,
    //       "Forward clearance range","NUM" },
    //     POPT_AUTOHELP
    //     { NULL, 0, 0, NULL, 0, NULL, NULL }
    // };

    // POpt popt(NULL, argc, argv, options, 0);
    // int c;
    // while((c = popt.getNextOpt()) >= 0) {}

    test_waypoints();

    ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 1, safeNavigate);
    ros::Subscriber sub_cur_pose = nh.subscribe("/vehicle_pose", 1, getCurrentPose);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
    ros::spin();
    return 0;
}
