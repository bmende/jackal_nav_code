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
#include "djikstra.h"

using namespace std;

ros::Publisher cur_pose_pub, sim_bot_path, vel_pub;

Pose sim_bot_pose, goal;
vector<geometry_msgs::PoseStamped> path_so_far;

GraphMap graph_map;
vector<Pose> plan;
int path_index = 0;

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
//    cur_pose_pub.publish(out_pose);

    graph_map.set_start(sim_bot_pose.x, sim_bot_pose.x);
}

void set_current_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    sim_bot_pose.x = msg->pose.position.x;
    sim_bot_pose.y = msg->pose.position.y;

    // get yaw! first get quaternion, then turn to yaw
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    sim_bot_pose.theta = yaw;

    cout << "current pose: " << sim_bot_pose << endl;
    graph_map.set_start(sim_bot_pose.x, sim_bot_pose.y);
}


void set_goal_pose(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;

    cout << "got goal: " << goal << endl;

    double distance_to_goal = goal.dist(sim_bot_pose);
    cout << "i am " << distance_to_goal << " away from the goal!\n";
    graph_map.set_goal(goal.x, goal.x);

    plan = graph_map.shortest_path();

}

void update_pose_from_vel(const geometry_msgs::Twist::ConstPtr& msg) {
    sim_bot_pose.update_heading(msg->angular.z);

    sim_bot_pose.x += msg->linear.x * cos(sim_bot_pose.theta);
    sim_bot_pose.y += msg->linear.x * sin(sim_bot_pose.theta);

    tf::Quaternion q = tf::createQuaternionFromYaw(sim_bot_pose.theta);
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();

    p.pose.position.x = sim_bot_pose.x;
    p.pose.position.y = sim_bot_pose.y;
    p.pose.position.z = 0;

    p.pose.orientation.x = q.getX();
    p.pose.orientation.y = q.getY();
    p.pose.orientation.z = q.getZ();
    p.pose.orientation.w = q.getW();

    //cur_pose_pub.publish(p);

    path_so_far.push_back(p);
    nav_msgs::Path disp_path;
    disp_path.poses = path_so_far;
    disp_path.header.frame_id = "map";
    disp_path.header.stamp = path_so_far[0].header.stamp;
    sim_bot_path.publish(disp_path);
}

pair<double, double> go_to_goal() {
    double distance_to_goal = goal.dist(sim_bot_pose);
    double heading = sim_bot_pose.heading_diff_to_pose(goal);
    cout << "i am " << distance_to_goal << ", " << heading << " away from the goal!\n";

    if (distance_to_goal < 0.25 || at_goal) {
        cout << "I am close enough\n";
        return make_pair(0, 0);
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

    return make_pair(for_vel, turn_vel);
}


pair<double, double> follow_map_plan() {
    double distance_to_goal = plan[path_index].dist(sim_bot_pose);
    double heading = sim_bot_pose.heading_diff_to_pose(plan[path_index]);
    cout << "i am " << distance_to_goal << ", " << (180*heading/PI) << " away from the next waypoint!\n";
    cout << plan[path_index] << endl;


    if (distance_to_goal < 0.5) {
        cout << "I am close enough\n";
        path_index++;
        return make_pair(0, 0);
    }

    double for_vel = 0, turn_vel = 0;
    if (abs(180*heading/PI) < 4.0) {
        cout << "forward!\n";
        for_vel = 0.5;
    } else if (heading < 0) {
        cout << "left!\n";
        for_vel = 0.07;
        turn_vel = -0.25;
    } else if (heading > 0) {
        cout << "right!\n";
        for_vel = 0.07;
        turn_vel = 0.25;
    }

    return make_pair(for_vel, turn_vel);
}

void unSafeNav(const sensor_msgs::JoyConstPtr& msg) {
    int R2 = msg->buttons[9];
    int R1 = msg->buttons[11];
    int X = msg->buttons[14];
    int O = msg->buttons[13];
    int triangle = msg->buttons[12];

    pair<double, double> desired_vel;
    if (triangle) {
        if (path_index < plan.size()) {
            desired_vel = follow_map_plan();
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = desired_vel.first;
            vel_msg.angular.z = desired_vel.second;
            vel_pub.publish(vel_msg);
        } else {
            cout << "Finally at the goal! Press X to reset!\n";
        }
    } else if (O) {
        graph_map.print_graph();
        graph_map.print_plan();
        at_goal = false;
    } else if (X) {
        cout << "replanning\n";
        plan = graph_map.shortest_path();
        path_index = 0;
    }

}

void initialize_map() {

    graph_map = GraphMap("src/orb_nav/src/eng_map.txt");
    graph_map.set_goal(30.2, 14.6);

    graph_map.print_graph();
    at_goal = false;

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "orb_nav_sim");
    ros::NodeHandle nh;


    cout << "I am here!!!" << endl;

    initialize_map();

    ros::Subscriber inital_pose = nh.subscribe("/initialpose", 1, set_starting_pose);
    ros::Subscriber current_pose = nh.subscribe("/vehicle_pose", 1, set_current_pose);
    ros::Subscriber goal_pose = nh.subscribe("/move_base_simple/goal", 1, set_goal_pose);
//    ros::Subscriber velocity_update = nh.subscribe("/jackal_velocity_controller/cmd_vel", 1, update_pose_from_vel);
    ros::Subscriber safeDriving = nh.subscribe("/bluetooth_teleop/joy", 1, unSafeNav);

    cur_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("orb_slam2/pose", 1);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
    sim_bot_path = nh.advertise<nav_msgs::Path>("vehicle_path", 1);

    //ros::spin();
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
//        go_to_goal();
        r.sleep();
    }

    return 0;

}
