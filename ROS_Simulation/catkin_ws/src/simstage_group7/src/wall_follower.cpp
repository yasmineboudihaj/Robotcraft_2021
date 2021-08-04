#include <iostream>
#include <numeric>


#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

/* 
 Idea:
 If you can turn left, do it.
 Else (if you can’t turn left), if you can continue going straight, just go straight.
 Else (if you can’t do either of the previous steps), if you can turn right, do it.
 If you reached a dead end, turn back by turning around (in either direction) 180 degrees.
 */

class WallFollower
{
protected:
    double left_obstacle_distance;
    double front_obstacle_distance;
    double right_obstacle_distance;

    double left_threshold = 0.3;
    double front_threshold = 0.3;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        if (front_obstacle_distance >= front_threshold){
            msg.linear.x = 0.1;
            msg.angular.z = 0.0;
        }
        if(left_obstacle_distance < left_threshold && front_obstacle_distance >= front_threshold){
          msg.linear.x = 0.1;
          msg.angular.z = 0.0;
        }
        else if(left_obstacle_distance < left_threshold ){
            msg.linear.x = 0.0;
            msg.angular.z = -0.7; // turn right     
        }
        /*else if (left_obstacle_distance == threshold) {
            msg.linear.x = 0.0;
            msg.angular.z = 0.5;
        }*/
        /*else {
            msg.linear.x = 0.0;
            msg.angular.z = 0.5;  
        }*/

        return msg;
    }

public:

    WallFollower(){
        // Initialize ROS
        ros::NodeHandle n;
        // Create a publisher object, able to push messages
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        laser_sub = n.subscribe("base_scan", 1000, &WallFollower::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        //left_obstacle_distance = *std::min_element (msg->ranges.begin() + 180 + 45, msg->ranges.begin() + 270 + 44);
        left_obstacle_distance = std::accumulate(msg->ranges.begin() + 270 - 30, msg->ranges.begin() + 270 + 29, 0) / 60.;
        front_obstacle_distance = std::accumulate (msg->ranges.begin() + 180 - 20, msg->ranges.begin() + 180 + 19, 0) / 40.;
        right_obstacle_distance = std::accumulate (msg->ranges.begin() + 45, msg->ranges.begin() + 90 + 44, 0) / 90.;
        std::cout << "Left: " << left_obstacle_distance;
        std::cout << ", front: " << front_obstacle_distance << std::endl;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            auto msg = calculateCommand();
            // Publish the new command
            cmd_vel_pub.publish(msg);

            // And throttle the loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "wall_follower");

    // Create our controller object and run it
    auto controller = WallFollower();
    controller.run();

    // And make good on our promise
    return 0;
}