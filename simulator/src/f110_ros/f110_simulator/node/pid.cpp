
#include <iostream>
#include <string>
#include <stdio.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
using namespace std;

//float angle_max = 0;
float yaw = 0;
float fix_dist = 0;
float e = 0;
float e_old = 0;
int min_index = 80;
int max_index = 1000;
float min_dist =0; // minimum distance from the wall
int min_ind = 0; // index of the minimum distance
float Kp;
float Kd;
float Ki;
float Ku = 3;
float Tu = 2;
float angle;
float L = 1.5/100;
float sum_err = 0;
float dist;
float mean = 0;
float contador = 1;
std::vector<float> range;



int get_min_index(std::vector<float> vec) {

    min_ind = std::distance(vec.begin(), std::min_element(vec.begin() + min_index, vec.begin() + max_index));

    return min_ind;
}

float get_min_dist(std::vector<float> vec,int minimum,int maximum){
    dist = *std::min_element(range.begin() + minimum, range.begin() + maximum);
    return dist;
}




void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    range = msg->ranges;

    min_dist = get_min_dist(range, min_index, max_index);

    get_min_index(range);

    fix_dist = (get_min_dist(range, 80, 540) + get_min_dist(range,540, 1000))/2;

    
    e = (fix_dist - min_dist - L * sin(angle));

    if (min_ind >= 80 && min_ind <= 540){
        Kp = 0.8* Ku;
        Ki = 0.01;
        Kd = 0.1*Ku*Tu;
        sum_err += e_old;

    } else{
        Kp = -0.8 * Ku;
        Ki = 0.01;
        Kd = -0.1*Ku*Tu;
        sum_err -= e_old;
    }

    yaw = (e * Kp + Ki * sum_err + Kd * (e_old - e));
    e_old = e;
    mean += yaw;
    contador++;

}

void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg){

    angle = (msg->drive).steering_angle;
    if ((msg->drive).speed == 0 ){
        sum_err = 0;
        mean = 0;
        contador = 0;
    }

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pid");
    ros::NodeHandle node;

    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/pid/drive", 1);
    ros::Subscriber lidar_sub = node.subscribe("/scan", 1, lidarCallback);
    ros::Subscriber drive_sub = node.subscribe("/drive", 1, driveCallback);
    ros::Rate loop_rate(100);
    ackermann_msgs::AckermannDrive drive_msg;

    drive_msg.speed = 1.5;
    while (ros::ok()) {
        drive_msg.steering_angle = yaw;

        std_msgs::Header header;
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        drive_st_msg.header = header;
        drive_st_msg.drive = drive_msg;
        command_pub.publish(drive_st_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}