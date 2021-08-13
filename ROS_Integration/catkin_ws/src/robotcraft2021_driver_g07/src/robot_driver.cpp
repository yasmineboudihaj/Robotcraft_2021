#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include "tf/transform_broadcaster.h"


class RobotDriver{

protected:
    // Publishers
    ros::Publisher odom_pub;
    ros::Publisher ir_front_sensor_pub;
    ros::Publisher ir_right_sensor_pub;
    ros::Publisher ir_left_sensor_pub;

    // Subscribers
    ros::Subscriber pose_sub;
    ros::Subscriber front_distance_sub;
    ros::Subscriber right_distance_sub;
    ros::Subscriber left_distance_sub;

    // Broadcasters
    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::Pose robot_pose;

    ros::Time current_time;
  
public:

    RobotDriver(){
        // Initialize ROS
        ros::NodeHandle n;

        // define nodes we publish to
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

        ir_front_sensor_pub = n.advertise<nav_msgs::Odometry>("/ir_front_sensor", 1000);
        ir_right_sensor_pub = n.advertise<nav_msgs::Odometry>("/ir_right_sensor", 1000);
        ir_left_sensor_pub  = n.advertise<nav_msgs::Odometry>("/ir_left_sensor",  1000);

        // define nodes we subscribe to
        pose_sub = n.subscribe("pose", 1000, &RobotDriver::pose_callback, this);

        front_distance_sub = n.subscribe("front_distance", 1000, &RobotDriver::front_distance_callback, this);
        right_distance_sub = n.subscribe("right_distance", 1000, &RobotDriver::right_distance_callback, this);
        left_distance_sub  = n.subscribe("left_distance",  1000, &RobotDriver::left_distance_callback,  this);
    }

    void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg){
        // convert geometry_msgs::Pose2D to geometry_msgs::Pose for odom
        robot_pose.position.x = msg->x;
        robot_pose.position.y = msg->y;
        // convert theta to quaternion
        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(msg->theta);
        robot_pose.orientation = orientation;

        // broadcast tf transform with pose information
        geometry_msgs::TransformStamped odom_trans;
        current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = msg->x;
        odom_trans.transform.translation.y = msg->y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = orientation;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
    }

    nav_msgs::Odometry create_odom_message(){
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = robot_pose.position.x;
        odom.pose.pose.position.y = robot_pose.position.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = robot_pose.orientation;

        return odom;
    }

    void front_distance_callback (const std_msgs::Float32::ConstPtr& msg){
        // TODO: store distance message
    }

    void right_distance_callback (const std_msgs::Float32::ConstPtr& msg){
        // TODO: store distance message
    }

    void left_distance_callback (const std_msgs::Float32::ConstPtr& msg){
        // TODO: store distance message
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            auto odom_msg = create_odom_message();
            odom_pub.publish(odom_msg);

            // TODO: publish sensor data
            
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