#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <dirent.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>




#define O_RDONLY   00
#define BILLION 1000000000.0L;

using std::stringstream;
using std::string;

// Globals
string kBagFileDirectory = "/home/jackal_user/bmende_cal/bags/";
string kBagNodeNameDirectory = "/home/jackal_user/Jackal/bag_node_name_bmende.txt";
const int kPrefixLength = 5;

ros::Publisher velocity_command_publisher;
// ros::Publisher stamped_velocity_command_publisher;

// Helper function that sends velocity commands to the Jackal to signal the
// start and end of the recording
void MoveJakcal (bool direction, double duration) {
    struct timespec ms1;
    struct timespec ms2;
    double s_elapsed1;

    geometry_msgs::Twist vel_command;
    vel_command.linear.x = 0;
    vel_command.linear.y = 0;
    vel_command.linear.z = 0;
    vel_command.angular.x = 0;
    vel_command.angular.y = 0;
    float rot_vel = 0.05;

    // rotate left
    if (direction) {
        vel_command.linear.x = -rot_vel;
        // rotate right
    } else {
        vel_command.linear.x = rot_vel;
    }

    clock_gettime(CLOCK_MONOTONIC, &ms1);
    clock_gettime(CLOCK_MONOTONIC, &ms2);
    s_elapsed1 = (ms2.tv_sec - ms1.tv_sec) + (ms2.tv_nsec - ms1.tv_nsec) / BILLION;

    while (s_elapsed1 < duration) {
        velocity_command_publisher.publish(vel_command);
        clock_gettime(CLOCK_MONOTONIC, &ms2);
        s_elapsed1 = (ms2.tv_sec - ms1.tv_sec) + (ms2.tv_nsec - ms1.tv_nsec) / BILLION;
    }

}

// Helper function that reads the names of the files in the bag files directory
// and figures out the name of the next bag file to be saved
void FindNextFileName (std::string* file_name) {

    char numbering[5];
    DIR* dirp = opendir(kBagFileDirectory.c_str());
    int latest_index = 0;
    struct dirent * dp;
    int max_prefix_number = 0;

    while ((dp = readdir(dirp)) != NULL){

        // Ignore the '.' and ".." directories
        if(!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) continue;
        for(int i = 0; i < kPrefixLength ; i++){
            numbering[i] = dp->d_name[i];
        }

        int prefix_number = atoi(numbering);
        if(prefix_number > max_prefix_number) max_prefix_number = prefix_number;
    }
    (void)closedir(dirp);

    char cur_prefix_number [5];
    sprintf(cur_prefix_number, "%05d", max_prefix_number + 1);
    *file_name = cur_prefix_number;
    file_name->append(".bag");
}

// Helper function that wraps the system command so that we can get the pid
// of the started process
pid_t system_wrapper(const char * command, int * infp, int * outfp)
{
    int p_stdin[2];
    int p_stdout[2];
    pid_t pid;

    if (pipe(p_stdin) == -1)
        return -1;

    if (pipe(p_stdout) == -1) {
        close(p_stdin[0]);
        close(p_stdin[1]);
        return -1;
    }

    pid = fork();

    if (pid < 0) {
        close(p_stdin[0]);
        close(p_stdin[1]);
        close(p_stdout[0]);
        close(p_stdout[1]);
        return pid;
    } else if (pid == 0) {
        close(p_stdin[1]);
        dup2(p_stdin[0], 0);
        close(p_stdout[0]);
        dup2(p_stdout[1], 1);
        dup2(open("/dev/null", O_RDONLY), 2);
        /// Close all other descriptors for the safety sake.
        for (int i = 3; i < 4096; ++i)
            close(i);

        setsid();
        execl("/bin/sh", "sh", "-c", command, NULL);
        _exit(1);
    }

    close(p_stdin[0]);
    close(p_stdout[1]);

    if (infp == NULL) {
        close(p_stdin[1]);
    } else {
        *infp = p_stdin[1];
    }

    if (outfp == NULL) {
        close(p_stdout[0]);
    } else {
        *outfp = p_stdout[0];
    }

    return pid;
}

// Helper function which starts and stops recording bag files by reading the
// corresponding signals from the joystick
void JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    static bool recording = false;
    static bool one_time_rec = false;
    static pid_t pid;

    int X = joy->buttons[14];
    int O = joy->buttons[13];

    // If X pressed, start recording
    if (X) {
        if (!recording) {
            recording = true;
            one_time_rec = true;
        }
    }

    // If O is pressed, stop recording
    if (O) {
        if (recording) {
            recording = false;
            stringstream command;
            stringstream command1;

            // Use the process id to kill the recording node: Not working, in order for
            // it to work you should send the command in sudo mode

            // command << "echo 7DigitPrecision | sudo -S kill -9 " << pid;
            // command << "kill 9 " << pid;
            // std::cout << command.str().c_str() << std::endl;
            // system(command.str().c_str());

            // Find out the name of the node recording the bag file and save it to a file
            command1 << "rosnode list | grep /record_* > " << kBagNodeNameDirectory.c_str();
            system(command1.str().c_str());
            std::ifstream in;
            string record_node_name;

            // Read the name of the recording node from file and kill it
            // NOTE: The path depends on the user name on the Jackal (jackal_user)
            in.open (kBagNodeNameDirectory.c_str(), std::ifstream::in);
            if(!in.is_open()){
                ROS_INFO("Error openning bag_node_name.txt");
            } else {
                if (std::getline(in, record_node_name)) {
                    command << "rosnode kill " << record_node_name.c_str();
                    system(command.str().c_str());

                    // After stopping recording, moves the Jackal to signal the end (rotates
                    // to the left)
                    MoveJakcal(true, 1.0);
                }
                in.close();
            }
        }
    }

    if (one_time_rec) {
        one_time_rec = false;
        // system("rosbag record -O ~/Jackal/bag_files/test1.bag "
        //                      "--duration=10 "
        // 	"/imu/data /cmd_vel /odometry/filtered &");
        int infp;
        int outfp;
        string bag_file_name;
        stringstream recording_command;
        FindNextFileName(&bag_file_name);

        recording_command << "rosbag record -o " << kBagFileDirectory.c_str() <<
            bag_file_name.c_str() <<
            " /webcam/left/camera_info /webcam/right/camera_info "
            " /webcam/left/image_raw/compressed/parameter_descriptions  /webcam/left/image_raw/compressed/parameter_updates "
            " /webcam/right/image_raw/compressed/parameter_descriptions  /webcam/right/image_raw/compressed/parameter_updates "
            " /webcam/left/image_raw/compressed "
            " /webcam/right/image_raw/compressed ";

        // Before starting recording, moves the Jackal to signal the start (rotates
        // to the right)
        MoveJakcal(false, 1.0);
        pid = system_wrapper(recording_command.str().c_str(), &infp, &outfp);
    }
    ROS_INFO("Recording status: %d", recording);
}


// void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd) {
// ros::Time time_stamp = ros::Time::now();
// geometry_msgs::TwistStamped cmd_vel_stamped;

// cmd_vel_stamped.twist.linear.x = cmd->linear.x;
// cmd_vel_stamped.twist.linear.y = cmd->linear.y;
// cmd_vel_stamped.twist.linear.z = cmd->linear.z;

// cmd_vel_stamped.twist.angular.x = cmd->angular.x;
// cmd_vel_stamped.twist.angular.y = cmd->angular.y;
// cmd_vel_stamped.twist.angular.z = cmd->angular.z;

// cmd_vel_stamped.header.stamp = time_stamp;

// stamped_velocity_command_publisher.publish(cmd_vel_stamped);
// }



int main(int argc, char **argv) {
    ros::init(argc, argv, "capture_data_bende");
    ros::NodeHandle nh;

    ros::Subscriber joystick_subscriber =
        nh.subscribe("/bluetooth_teleop/joy",5, JoystickCallback);

    // ros::Subscriber cmd_vel_subscriber =
    //   nh.subscribe("/cmd_vel",5, CmdVelCallback);

    // stamped_velocity_command_publisher =  nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped", 5);

    velocity_command_publisher =  nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("Press X to start recording and press O to stop recording.");
    ROS_INFO("Jackal rotates to the right to signal the start of recording"
             " and rotates to the left to signal the end of recording.");

    ros::spin();
    return 0;
}
