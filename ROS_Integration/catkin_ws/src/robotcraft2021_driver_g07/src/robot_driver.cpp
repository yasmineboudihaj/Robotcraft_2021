#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

class RobotDriver{

protected:
    ros::Publisher odom_pub;
    ros::Subscriber pose_sub;

    geometry_msgs::Pose robot_pose;
public:

    RobotDriver(){
        // Initialize ROS
        ros::NodeHandle n;

        // define nodes we publish to and subscribe from
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
        pose_sub = n.subscribe("pose", 1000, &RobotDriver::pose_callback, this);
    }

    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg){
        robot_pose.position.x = msg->x;
        robot_pose.position.y = msg->y;
        // TODO:convert theta to quaternion

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            //auto msg = chooseBehavior();
            //cmd_vel_pub.publish(msg);

            // And throttle the loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};

int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "robot_driver_node");

    // Create our controller object and run it
    auto driver = RobotDriver();
    driver.run();

    // And make good on our promise
    return 0;
}