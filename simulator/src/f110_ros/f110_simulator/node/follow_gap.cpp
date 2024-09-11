
#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
using namespace std;


float yaw = 0;
float speed = 4;
int min_index = 270;
int max_index = 810;
int ind =0;

std::vector<float> range;


float get_max_angle(std::vector<float> vec) {
    /**
    * Calculates the angle corresponding to the maximum distance in the LIDAR scan data.
    * 
    * @param vec A vector containing LIDAR scan data (distances).
    * @return The angle in radians corresponding to the maximum distance within the specified index range.
    */
    int max; //max index in a array
    float angle;

    max = std::distance(vec.begin(), std::max_element(vec.begin() + min_index, vec.begin() + max_index));
    angle = max * 2 *M_PI/1080;

    return angle;

}

float get_min_angle(std::vector<float> vec) {
    /**
     * Calculates the angle corresponding to the minimum distance in the LIDAR scan data.
     * 
     * @param vec A vector containing LIDAR scan data (distances).
     * @return The angle in radians corresponding to the minimum distance within the specified index range.
     */

    int min; //min index in a array
    float angle;

    min = std::distance(vec.begin(), std::min_element(vec.begin() + min_index, vec.begin() + max_index));
    angle = min * 2*M_PI/1080;

    return angle;
}




void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    /**
     * Callback function to process incoming LIDAR scan data.
     * 
     * This function calculates the yaw (steering angle) based on the LIDAR scan data
     * and adjusts the yaw to avoid obstacles.
     * 
     * @param msg A pointer to the incoming LIDAR scan message.
     */

    float angle;
    float angle_min;
    float Kp;
    float Kp2;


    range = msg->ranges;
    angle = get_max_angle(range);
    angle_min = get_min_angle(range) - M_PI;
    yaw = angle -M_PI;
    Kp = 0.2;
    Kp2 = 0.4;
    

    float min_distance = *std::min_element(range.begin() + min_index, range.begin() + max_index);

    if (min_distance < 1) { // Assuming 0.5 meters as too close
        yaw =yaw -Kp*angle_min;

        if (min_distance < 0.2) {
            yaw =yaw -Kp2*angle_min;
        }
    }
    
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follow_gap");
    ros::NodeHandle node;

    ros::Publisher command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/follow_gap/drive", 1);
    ros::Subscriber lidar_sub = node.subscribe("/scan", 1, callback);
    ros::Rate loop_rate(50);
    ackermann_msgs::AckermannDrive drive_msg;

    while (ros::ok()) {
        drive_msg.steering_angle = yaw;

        drive_msg.speed = speed;

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
