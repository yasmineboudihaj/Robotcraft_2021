#include <iostream>
#include <numeric>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <tf/tf.h>

class WallFollower
{
protected:
    double left_obstacle_distance;
    double front_obstacle_distance;
    double right_obstacle_distance;

    // threshold for the distances to the wall
    double d = 25;
    // threshold to avoid hitting the wall
    double emergency_threshold = 15;

    // linear and angular velocity
    double linear_vel = 0.1;
    double angular_vel = 0.8;

    ros::Publisher cmd_vel_pub;
    ros::Subscriber left_sensor_sub;
    ros::Subscriber front_sensor_sub;
    ros::Subscriber right_sensor_sub;
    /*
        The idea behind the wall follower algorithm, is to find the way out of a labyrinth
        by following the right wall.
        In the following, we define three behaviors to achieve this:
          1) findWall():      If there is no wall in sight, we find the right wall by
                              going straight while slightly turning right.
          2) turnLeft():      If we cannot go straight because a wall is blocking our
                              way, we turn left until the path is clear again.
          3) followTheWall(): Once we have find the right wall, and as long as nothing
                              is blocking our way, we go straight.
    */
    geometry_msgs::Twist findWall()
    {
        ROS_INFO("Finding the wall");
        auto msg = geometry_msgs::Twist();
        msg.linear.x = linear_vel;
        msg.angular.z = -1. * angular_vel;
        return msg;
    }

    geometry_msgs::Twist turnLeft()
    {
        ROS_INFO("Turning left");
        auto msg = geometry_msgs::Twist();
        msg.angular.z = angular_vel;
        return msg;
    }

    geometry_msgs::Twist followTheWall()
    {
        ROS_INFO("Following the wall");
        // we want the robot to always turn a full 90 degrees before continuing
        // to go straight (to avoid collisions with walls).
        // So, in this if-condition, we correct the robot's rotation.
        auto msg = geometry_msgs::Twist();
        if (right_obstacle_distance < emergency_threshold){
            ROS_INFO("Correcting");
            msg.angular.z = angular_vel;
        } else {
            msg.linear.x = linear_vel;
        }
        return msg;
    }

    // Chooses one of the three previously defined behaviors based on the current
    // situation (i.e., is or is there not a wall on the left/front/right?)
    geometry_msgs::Twist chooseBehavior()
    {
        if (front_obstacle_distance >  d && left_obstacle_distance > d && right_obstacle_distance > d)
        {
            // no walls in sight (gotta find the right wall!)
            return findWall();
        }
        else if (front_obstacle_distance < d && left_obstacle_distance > d && right_obstacle_distance > d)
        {
            // wall only on the front (cannot continue to go straight)
            return turnLeft();
        }
        else if (front_obstacle_distance > d && left_obstacle_distance > d && right_obstacle_distance < d)
        {
            // wall only on the right (found right wall)
            return followTheWall();
        }
        else if (front_obstacle_distance > d && left_obstacle_distance < d && right_obstacle_distance > d)
        {
            // wall only on the left (we lost our right wall)
            return findWall();
        }
        else if (front_obstacle_distance < d && left_obstacle_distance > d && right_obstacle_distance < d)
        {
            // walls front and right (cannot go straight or right, so let's find another way)
            return turnLeft();
        }
        else if (front_obstacle_distance < d && left_obstacle_distance < d && right_obstacle_distance > d)
        {
            // walls front and left
            return turnLeft();
        }
        else if (front_obstacle_distance < d && left_obstacle_distance < d && right_obstacle_distance < d)
        {
            // dead-end (walls front, left and right)
            return turnLeft();
        }
        else if (front_obstacle_distance > d && left_obstacle_distance < d && right_obstacle_distance < d)
        {
            // wall left and right
            return turnLeft();
        }
    }

public:

    WallFollower(){
        // Initialize ROS
        ros::NodeHandle n;

        // define nodes we publish to and subscribe from
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        left_sensor_sub  = n.subscribe("ir_left_sensor", 100, &WallFollower::leftSensorCallback,  this);
        front_sensor_sub = n.subscribe("ir_front_sensor", 100, &WallFollower::frontSensorCallback, this);
        right_sensor_sub = n.subscribe("ir_right_sensor", 100, &WallFollower::rightSensorCallback, this);
    }

    void leftSensorCallback(const sensor_msgs::Range::ConstPtr& msg){
        left_obstacle_distance = msg->range;
    }

    void frontSensorCallback(const sensor_msgs::Range::ConstPtr& msg){
        front_obstacle_distance = msg->range;
    }

    void rightSensorCallback(const sensor_msgs::Range::ConstPtr& msg){
        right_obstacle_distance = msg->range;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            auto msg = chooseBehavior();
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
