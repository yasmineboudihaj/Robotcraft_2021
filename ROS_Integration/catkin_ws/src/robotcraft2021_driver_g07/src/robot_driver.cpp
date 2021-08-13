#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>


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
        // convert theta to quaternion
        robot_pose.orientation = tf::createQuaternionMsgFromYaw(msg->theta);
    }

    nav_msgs::Odometry create_odom_message(){
        nav_msgs::Odometry odom;
        //odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = robot_pose.position.x;
        odom.pose.pose.position.y = robot_pose.position.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = robot_pose.orientation;

        return odom;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            auto odom_msg = create_odom_message();
            odom_pub.publish(odom_msg);

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