#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <unistd.h>

#define STATE_LOST 0
#define STATE_CCW 1
#define STATE_WALL1 2
#define STATE_WALL2 3

bool right_sensor,left_sensor;

//Publishers
ros::Publisher cmd_vel_pub;

//Sensor's Callback functions

void right_sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(msg->ranges[0] < msg->range_max)
        right_sensor = true;
    else
        right_sensor = false;
}
void left_sensor_callback (const sensor_msgs::LaserScan::ConstPtr& msg){
    if(msg->ranges[0] < msg->range_max)
        left_sensor = true;
    else
        left_sensor = false;
}

// Move the robot forward
void move (double linear_vel){
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = linear_vel;
    cmd_vel_pub.publish(cmd_msg);
}

//Turn
void turn (double angular_vel){
    geometry_msgs::Twist cmd_msg;
    cmd_msg.angular.z = angular_vel;
    cmd_vel_pub.publish(cmd_msg);
}

void slight_turn (double angular_vel){
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0.1;
    cmd_msg.angular.z = angular_vel;
    cmd_vel_pub.publish(cmd_msg);
}

// Functions for the 'different possible states we can have which are:
// Lost, CCW, Right_Wall, Left_Wall

int lost(){
    if (!right_sensor && !left_sensor)
        move(0.1);
    else
        return STATE_CCW;
    return STATE_LOST;
}

int ccw(){
    if (right_sensor || left_sensor)
        turn(0.2);
    else
        return STATE_WALL1;
    return STATE_CCW;
}

int wall1(){
    if (!right_sensor){
        /*move(0.1);
        usleep(100000);
        turn(-0.2);
        usleep(100000);*/
        slight_turn(-0.2);
    }
    else
        return STATE_WALL2;
    return STATE_WALL1;
}
int wall2(){
    if (!left_sensor){
        if (right_sensor){
            /*turn(0.2);
            usleep(100000);
            move(0.1);
            usleep(100000);*/
            slight_turn(0.35);
        } else {
            return STATE_WALL1;
        }
    } else {
        return STATE_CCW;
    }
    return STATE_WALL2;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "maze_BASICsolver");
    ros::NodeHandle n;
    ros::Rate loop_rate(10); //10 Hz

    //Publish cmd_vel
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);

    //Subcribing to both left and right sensors
    ros::Subscriber right_sensor_sub; 
    ros::Subscriber left_sensor_sub; 

    right_sensor_sub = n.subscribe("base_scan_3", 100, right_sensor_callback);
    left_sensor_sub  = n.subscribe("base_scan_2", 100, left_sensor_callback);


    // At first the robot is Lost
    int state = STATE_LOST; 

    while (ros::ok()) {
        switch (state){
            case STATE_LOST :
                ROS_INFO ("Lost");
                state = lost();
                break;
            case STATE_CCW :
                ROS_INFO ("CCW");
                state = ccw();
                break;
            case STATE_WALL1 : 
                ROS_INFO ("WALL 1");
                state = wall1();
                break;
            case STATE_WALL2 :
                ROS_INFO("WALL 2");
                state = wall2();
                break;
            default : 
                ROS_ERROR("Invalid_State");
                state = STATE_LOST;
            }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}