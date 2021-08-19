#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

#include <tf/tf.h>

class SquareTest
{
protected:
    double x, y, theta;
    double goal_distance;
    double goal_theta = M_PI/2.;

    int state = 0;

    // Publishers
    ros::Publisher cmd_vel_pub;

    // Subscribers
    ros::Subscriber odom_sub;

public:

    SquareTest(){
        // Initialize ROS
        ros::NodeHandle n;

        // define topics we publish to
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

        // define topics we subscribe from
        odom_sub = n.subscribe("odom", 1000, &SquareTest::odom_callback, this);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        
        // Convert quaternion to Euler angle
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        theta = yaw;
    }

    double get_distance(double x1, double x2, double y1, double y2){
        return sqrt(pow(x1 - y1, 2) + pow(x2 - y2, 2));
    }

    geometry_msgs::Twist calculateTwist(){
        geometry_msgs::Twist msg;

        switch(state) {
        case 0:
            // go straight 1
            msg.linear.x  = 5.;
            msg.angular.z = 0.;
            if (x > 0.5) state = 1;
            break;
        case 1:
            // turning 1
            msg.linear.x  = 0.;
            msg.angular.z = -5.;
            if (theta > M_PI/2.) state = 2;
            break;
        case 2:
            // go straight 2
            msg.linear.x  = 5.;
            msg.angular.z = 0.;
            if (y > 0.5) state = 3;
            break;
        case 3:
            // turning 2
            msg.linear.x  = 0.;
            msg.angular.z = -5.;
            if (theta > M_PI) state = 4;
        case 4:
            // go straight 3
            msg.linear.x  = 5.;
            msg.angular.z = 0.;
            if (x < 0.5) state = 5;
            break;
        case 5:
            // turning 3
            msg.linear.x  = 0.;
            msg.angular.z = -5.;
            if (theta > 1.5 * M_PI) state = 6;
            break;
        case 6:
            // go straight 4
            msg.linear.x  = 5.;
            msg.angular.z = 0.;
            if (y < 0) state = 7;
            break;
        case 7:
            // turning 4
            msg.linear.x  = 0.;
            msg.angular.z = -5.;
            if (theta > 2. * M_PI - M_PI/16.) state = 0;
            break;
        default:
            // some error occurred
            break;
        }
        return msg;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {

            // And throttle the loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};

int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "square_test");

    // Create our controller object and run it
    auto controller = SquareTest();
    controller.run();

    // And make good on our promise
    return 0;
}