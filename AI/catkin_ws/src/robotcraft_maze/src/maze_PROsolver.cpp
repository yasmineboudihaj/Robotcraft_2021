#include <vector>
#include <unistd.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h" // maybe delete this later...
#include "sensor_msgs/LaserScan.h" // maybe delete this later...
#include "nav_msgs/OccupancyGrid.h"


class WavefrontPlanner
{
protected:
    std::vector<int8_t> map;

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
    std::vector<int> robot_pos = {5, 495};
    std::vector<int> goal_pos  = {135, 0};

    // Step 3: define adjacent cells matrix, the costmap and O
    std::vector<std::vector<int>> adjacent_cells = {{1, 0},
                                                    {0, 1},
                                                    {-1, 0},
                                                    {0, -1}};
    
    
    std::vector<std::vector<int>> costmap;

    // initialize o with target point
    std::vector<std::vector<int>> o = {{goal_pos[0], goal_pos[1]}};
    

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
        map = msg->data;
        std::cout << "Length of vector = " << map.size() << std::endl;
    }
    /*while (dim O > 0)
        for each r row of A
            aux(1,1: 2) = O(1,1: 2) + A(r, 1: 2)
            if 1 ≤ aux 1,1 ≤ dim x axis and 1 ≤ aux 1,2 ≤ dim y axis and M aux = 0 and C aux = 0
                C aux, 1: 2 = C O 1,1: 2 + 1
                O dim O + 1,1: 2 = aux
        O = O(2: dim O , 1: 2)*/
    void algorithm(){

        std::cout << "Start algorithm!" << std::endl;
        if (map.size() == 0){
            std::cout << "Map is empty" << std::endl;
            return;
        }
        while(o.size() != 0){
            std::cout << "Got inside while" << std::endl;
            std::vector<std::vector<int>> tmp = {};
            for(int row = 0; row < adjacent_cells.size(); row++){
                for (int o_index = 0; o_index < o.size(); o_index++){
                    std::vector<int> aux = {o[o_index][0] + adjacent_cells[row][0],
                                            o[o_index][1] + adjacent_cells[row][1]};
                // TODO: make sure the map index is correct!!
                std::cout << "Index for map: " << aux[0] * 540 + aux[1] << std::endl;
                std::cout << "Size of map: " << map.size() << std::endl;
                std::cout << "...gives us:" << map[aux[0] * 540 + aux[1]] << std::endl;
                std::cout << "Indices for costmap: " << aux[0] << "&" << aux[1] << std::endl;
                std::cout << "...gives us: " << costmap[aux[0]][aux[1]] << std::endl;
                if ((0 <= aux[0] && aux[0] < 540)     &&
                    (0 <= aux[1] && aux[1] < 540)     &&
                    ((int)map[aux[0] * 540 + aux[1]] == 0) && 
                    (costmap[aux[0]][aux[1]] == 0)){
                        std::cout << "Got into if!" << std::endl;
                        costmap[aux[0]][aux[1]] = costmap[o[0][0]][o[0][1]] + 1;
                        tmp.push_back(aux);
                    }
                }
                o = tmp;
            }
        }
        std::cout << "Stop algorithm!" << std::endl;
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            //cmd_vel_pub.publish(msg);
            algorithm();
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