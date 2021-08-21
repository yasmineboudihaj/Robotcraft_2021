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

    /*
      Step 2: Identify the robot start position (i.e., (0.05/4.95)) and the
              target position (i.e., (1.35, 0)) within the x-y-grid:
              The map server gives us an occupancy grid of dimension 540x540,
              we know that the map itself is 5.4x5.4 units. This means each pixel
              of the grid is 0.01 units wide and high. 
              Thus, we get the correct values directly by multiplying the map
              coordinates by 100 to get the corresponding coordinate in the x-y-grid.
    */
    std::vector<float> robot_pos = {5, 495};
    std::vector<float> goal_pos  = {135, 0};

    // Step 3: define adjacent cells matrix, the costmap and O
    std::vector<std::vector<int>> adjacent_cells = {{1, 0},
                                                    {0, 1},
                                                    {-1, 0},
                                                    {0, -1}};
    
    std::vector<std::vector<int>> costmap;
    std::vector<int> o;

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

        // Step 3: Initialize grid point with target with 1
        costmap = std::vector<std::vector<int>>( 540 , std::vector<int> (540, 0.));
        costmap[goal_pos[0]][goal_pos[1]] = 1;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    }

    // Step 1: Discretize the (known) map: 0 = empty, 100 = full
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