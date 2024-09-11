
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

using namespace std;

float ttc_threshold = 0.3; // Adjust this threshold based on testing
float speed = 0;
std::vector<float>  range;


void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg, ros::Publisher* stop_pub) {
    /**
     * Callback function to process incoming LIDAR scan data and calculate time-to-collision (TTC).
     * 
     * This function checks each LIDAR reading to determine if a collision is imminent based on the TTC.
     * If a collision is imminent, it publishes a stop signal.
     * 
     * @param msg A pointer to the incoming LIDAR scan message.
     * @param stop_pub A pointer to the ROS publisher for the stop signal.
     */
    bool collision_imminent = false;
    float beta = 2 * M_PI / 1080;
    float distance = 0;
    float angle = 0;
    float projected_speed = 0;
    float ttc = 0;
    range = msg->ranges;
    
    for (size_t i = 0; i < range.size(); ++i) {
        distance = range[i];
        angle = i * beta;
        projected_speed = speed * cos(angle);
        ttc = abs(distance / projected_speed);
        if (ttc < ttc_threshold) {
            collision_imminent = true;
            break;
        }
    }

    std_msgs::Bool stop_msg;
    stop_msg.data = collision_imminent;
    stop_pub->publish(stop_msg);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    speed = msg->twist.twist.linear.x; // Update the global 'speed' variable with the current linear speed
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "aeb");
    ros::NodeHandle nh;

    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/aeb/stop", 1);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(lidarCallback, _1, &stop_pub));
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    ros::spin();

    return 0;

}