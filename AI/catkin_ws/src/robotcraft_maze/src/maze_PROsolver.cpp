#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // maybe delete this later...
#include "sensor_msgs/LaserScan.h" // maybe delete this later...
#include "nav_msgs/OccupancyGrid.h"

class WavefrontPlanner
{
protected:
    std::vector<int8_t> grid;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber occupancy_grid_sub;

    geometry_msgs::Twist findWall()
    {
        auto msg = geometry_msgs::Twist();
        msg.linear.x = 0.2;
        msg.angular.z = -0.3;
        return msg;
    }

public:

    WavefrontPlanner(){
        // Initialize ROS
        ros::NodeHandle n;

        // define nodes we publish to and subscribe from
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        laser_sub   = n.subscribe("base_scan", 1000, &WavefrontPlanner::laserCallback, this);
        occupancy_grid_sub = n.subscribe("map", 1000, &WavefrontPlanner::occupancyGridCallback,  this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    }

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        grid = msg->data;
        std::cout << "Length of vector = " << grid.size() << std::endl;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            //cmd_vel_pub.publish(msg);

            // And throttle the loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};

int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "maze_PROsolver_node");

    // Create our planner object and run it
    auto planner = WavefrontPlanner();
    planner.run();

    // And make good on our promise
    return 0;
}