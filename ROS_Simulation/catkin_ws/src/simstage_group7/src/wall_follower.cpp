#include <iostream>

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
private:
    double obstacle_distance;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        if(this->obstacle_distance < 1 ){
          msg.linear.x = 0.1;
          msg.angular.z = 0.0;
          
        }else{
            msg.linear.x = 0;
            msg.angular.z = 0;
        }

        return msg;
    }

public:

    WallFollower(){
        // Initialize ROS
        ros::NodeHandle n;
        // Create a publisher object, able to push messages
        this->cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        this->laser_sub = n.subscribe("base_scan", 1000, &WallFollower::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        this->obstacle_distance = *std::min_element (msg->ranges.begin() + 180, msg->ranges.begin() + 359);
        std::cout << "Min: " << this->obstacle_distance << std::endl;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            auto msg = calculateCommand();
            // Publish the new command
            this->cmd_vel_pub.publish(msg);

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