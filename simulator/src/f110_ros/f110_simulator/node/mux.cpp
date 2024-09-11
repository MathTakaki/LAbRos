#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <thread>


std::string current_mode = "neutro";
float angle = 0;
float speed = 0;
bool aeb_active = false;
bool moving_backward = false;
ros::Time aeb_activation_time;
const ros::Duration backward_duration(2); // Duration for moving backward


void modeCallback(const std_msgs::String::ConstPtr& msg) {
    std::string mode = msg->data;
    if (mode == "k") {
        current_mode = "manual";
    } else if(mode == "f"){
        current_mode = "follow_the_gap";
    } else if(mode == "p"){
        current_mode = "pid";
    }
    ROS_INFO("Mode changed to: %s", current_mode.c_str());
}

void keyboardCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, ros::Publisher* drive_pub) {
    if (current_mode == "manual" && !aeb_active) {

    speed = (msg->drive).speed;
    angle = (msg->drive).steering_angle;
    }
}


void followGapCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, ros::Publisher* drive_pub) {
    if (current_mode == "follow_the_gap") {
  
    speed = (msg->drive).speed;
    angle = (msg->drive).steering_angle;
    }
}

void aebCallback(const std_msgs::Bool::ConstPtr& msg) {
    aeb_active = msg->data;
    if (aeb_active && current_mode == "manual") {
        speed = -1;  
        angle = 0;
        ROS_WARN("AEB activated: Stopping the car!");
        moving_backward = true;
        aeb_activation_time = ros::Time::now();  

    }
}
void pidCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg, ros::Publisher* drive_pub) {
    if (current_mode == "pid") {
    speed = (msg->drive).speed;
    angle = (msg->drive).steering_angle;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mux");

    ros::NodeHandle nh;

    ros::Publisher drive_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    ros::Subscriber mode_sub = nh.subscribe("/mode", 1, modeCallback);
    ros::Subscriber pid_sub = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/pid/drive", 1, boost::bind(pidCallback, _1, &drive_pub));
    ros::Subscriber keyboard_sub = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/keyboard/drive", 1, boost::bind(keyboardCallback, _1, &drive_pub));

    ros::Subscriber follow_gap_sub = nh.subscribe<ackermann_msgs::AckermannDriveStamped>("/follow_gap/drive", 1, boost::bind(followGapCallback, _1, &drive_pub));
    ros::Subscriber aeb_sub = nh.subscribe("/aeb/stop", 1, aebCallback);

    ros::Rate loop_rate(50);

  
    while(ros::ok()) {
    // Make and publish message
    //  Header

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = angle;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    drive_pub.publish(drive_st_msg);
    if (current_mode == "manual"){
    while (aeb_active) {
        if (moving_backward) {
            if ((ros::Time::now() - aeb_activation_time) > backward_duration) {
                    speed = 0;
                    angle = 0;
                    moving_backward = false;
                    aeb_active = false;
                    ROS_INFO("Completed backward maneuver after AEB activation.");
                }
            }
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
