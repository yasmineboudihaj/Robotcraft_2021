// "reactive_navigation" node: subscribes laser data and publishes
// velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
}

int main(int argc, char **argv){
    
  ros::init(argc, argv, "reactive_navigation");
  
  ros::NodeHandle n;

  //Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  ros::Rate loop_rate(10); //10 Hz

  //initializations:
  geometry_msgs::Twist cmd_vel_msg;

  while (ros::ok()){
    
      if(obstacle_distance < 1.0){
          cmd_vel_msg.linear.x = 0.0;
          cmd_vel_msg.angular.z = 0.0;
          
      }else{
            cmd_vel_msg.linear.x = 0.9;
            cmd_vel_msg.angular.z = 0.3;
      }

    //publish velocity commands:
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
