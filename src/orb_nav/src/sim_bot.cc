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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "helpers.h"

using namespace std;

ros::Publisher cur_pose_pub, sim_bot_path, vel_pub;

Pose sim_bot_pose, goal;
vector<geometry_msgs::PoseStamped> path_so_far;

bool at_goal = true;


void set_starting_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    sim_bot_pose.x = msg->pose.pose.position.x;
    sim_bot_pose.y = msg->pose.pose.position.y;

        // get yaw! first get quaternion, then turn to yaw
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    sim_bot_pose.theta = yaw;

    cout << "initial pose: " << sim_bot_pose << endl;

    geometry_msgs::PoseStamped out_pose;
    out_pose.header = msg->header;
    out_pose.pose = msg->pose.pose;
    cur_pose_pub.publish(out_pose);

    path_so_far.clear();


}

void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;

    cout << "got goal: " << goal << endl;

    double distance_to_goal = goal.dist(sim_bot_pose);
    cout << "i am " << distance_to_goal << " away from the goal!\n";
    at_goal = false;

}

void update_pose_from_vel(const geometry_msgs::Twist::ConstPtr& msg) {
    sim_bot_pose.update_heading(msg->angular.z);

    sim_bot_pose.x += msg->linear.x * cos(sim_bot_pose.theta);
    sim_bot_pose.y += msg->linear.x * sin(sim_bot_pose.theta);

    tf::Quaternion q = tf::createQuaternionFromYaw(sim_bot_pose.theta);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "ground";
    p.header.stamp = ros::Time::now();

    p.pose.position.x = sim_bot_pose.x;
    p.pose.position.y = sim_bot_pose.y;
    p.pose.position.z = 0;

    p.pose.orientation.x = q.getX();
    p.pose.orientation.y = q.getY();
    p.pose.orientation.z = q.getZ();
    p.pose.orientation.w = q.getW();

    cur_pose_pub.publish(p);

    path_so_far.push_back(p);
    nav_msgs::Path disp_path;
    disp_path.poses = path_so_far;
    disp_path.header.frame_id = "ground";
    disp_path.header.stamp = path_so_far[0].header.stamp;
    sim_bot_path.publish(disp_path);
}

void go_to_goal() {
    if (at_goal) {
        return;
    }
    double distance_to_goal = goal.dist(sim_bot_pose);
    double heading = sim_bot_pose.heading_diff_to_pose(goal);
    cout << "i am " << distance_to_goal << ", " << heading << " away from the goal!\n";

    if (distance_to_goal < 0.25) {
        at_goal = true;
        cout << "I am close enough\n";
    }

    bool spin_or_go = false;
    double for_vel = 0, turn_vel = 0;
    if (abs(heading) < 0.06) {
        cout << "forward!\n";
        for_vel = min(0.1, distance_to_goal);
    } else if (heading < 0) {
        cout << "left!\n";
        turn_vel = -0.05;
    } else if (heading > 0) {
        cout << "right!\n";
        turn_vel = 0.05;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = for_vel;
    vel_msg.angular.z = turn_vel;
    vel_pub.publish(vel_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "orb_nav_sim");
    ros::NodeHandle nh;


    cout << "I am here!!!" << endl;

    ros::Subscriber inital_pose = nh.subscribe("/initialpose", 1, set_starting_pose);
    ros::Subscriber goal_pose = nh.subscribe("/move_base_simple/goal", 1, set_goal_pose);
    ros::Subscriber velocity_update = nh.subscribe("/jackal_velocity_controller/cmd_vel", 1, update_pose_from_vel);

    cur_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
    sim_bot_path = nh.advertise<nav_msgs::Path>("vehicle_path", 1);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        go_to_goal();
        r.sleep();
    }

    return 0;

}
