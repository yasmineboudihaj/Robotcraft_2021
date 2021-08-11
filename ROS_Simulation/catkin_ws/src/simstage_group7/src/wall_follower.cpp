#include <iostream>
#include <numeric>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

class WallFollower
{
protected:
    double left_obstacle_distance;
    double front_obstacle_distance;
    double right_obstacle_distance;

    // threshold for the distances to the wall
    double d = 0.9;

    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    double current_theta = 0;
    
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
        auto msg = geometry_msgs::Twist();
        msg.linear.x = 0.2;
        msg.angular.z = -0.3;
        return msg;
    }

    geometry_msgs::Twist turnLeft()
    {
        auto msg = geometry_msgs::Twist();
        msg.angular.z = 0.3;
        return msg;
    }

    geometry_msgs::Twist followTheWall()
    {
        // we want the robot to always turn a full 90 degrees before continuing
        // to go straight (to avoid collisions with walls).
        // So, in this if-condition, we correct the robot's rotation.
        auto msg = geometry_msgs::Twist();
        if ((current_theta > 0                     && current_theta < M_PI/2. - M_PI/16.   )|| 
            (current_theta > M_PI/2. + M_PI/16.    && current_theta < M_PI - M_PI/16.      )||
            (current_theta > M_PI + M_PI/16.       && current_theta < 1.5 * M_PI - M_PI/16.)||
            (current_theta > 1.5 * M_PI + M_PI/16. && current_theta < 2 * M_PI - M_PI/16.  ))
        {
            //std::cout << "Correcting" << std::endl;
            msg.angular.z = 0.3;
        }
        else
        {
            msg.linear.x = 0.3;
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
        laser_sub   = n.subscribe("base_scan", 1000, &WallFollower::laserCallback, this);
        odom_sub    = n.subscribe("odom",      1000, &WallFollower::odomCallback,  this);
    }

    /*
        Calculates the mean distances to a wall of a region using the data from the
        lidar sensor. There are three regions:
        The front left region (120 to 160 deg), the front region (160 to 200 deg) and 
        the front right region (200 to 240 deg).
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        left_obstacle_distance  = std::accumulate(msg->ranges.begin() + 200, msg->ranges.begin() + 239, 0) / 40.;
        front_obstacle_distance = std::accumulate(msg->ranges.begin() + 160, msg->ranges.begin() + 199, 0) / 40.;
        right_obstacle_distance = std::accumulate(msg->ranges.begin() + 120, msg->ranges.begin() + 159, 0) / 40.;
        //std::cout  << left_obstacle_distance << ", " << front_obstacle_distance << ", " << right_obstacle_distance << std::endl;
    }
    
    // saves the current orientation of the robot taken from the odometry topic
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
    {
        auto odom_pose = odometry_msg -> pose;
        current_theta = abs(odom_pose.pose.orientation.z);  

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