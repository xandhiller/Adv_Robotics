#include "astar.h"
#include "occupancy_graph.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <ros/package.h>
#include <string>
#include <vector>
#include <tuple>
#include <sstream>
#include "structures.h"


// To allows student tests
#include "open_list.h"
#include "extended_list.h"


// Function prototypes
nav_msgs::Path PathToNavMsgPath(std::vector<astar_tutorial::RealCoordinates > path);
std::vector<astar_tutorial::RealCoordinates > SmoothPath(std::vector<astar_tutorial::RealCoordinates > path,
        double data_weight, double smooth_weigth, double tolerance);



/* Main */
int main(int argc, char** argv){

    // Ros node init
    ros::init(argc, argv, "astar_tutorial");
    ros::NodeHandle nh("~");

    // Declaration of the parameters
    std::string map_file_name;
    std::string color_cost_string;
    double cell_size, map_width, wall_clearance, astar_lambda, start_x, start_y, end_x, end_y;
    double smooth_data_weight, smooth_tolerance, smooth_distance_weight;
    bool euclidean_distance, diagonal_movement;
   
    // Read the parameters from the launch file
    nh.param<std::string>("map_file_name", map_file_name, "map.png");
    nh.param<std::string>("color_cost", color_cost_string, std::string());
    nh.param<double>("cell_size", cell_size, 0.1);
    nh.param<double>("map_width", map_width, 10.0);
    nh.param<double>("wall_clearance", wall_clearance, 0.0);
    nh.param<double>("astar_lambda", astar_lambda, 1.0);
    nh.param<double>("start_x", start_x, 1.0);
    nh.param<double>("start_y", start_y, 4.0);
    nh.param<double>("end_x", end_x, 9);
    nh.param<double>("end_y", end_y, 1);
    nh.param<double>("smooth_tolerance", smooth_tolerance, 0.001);
    nh.param<double>("smooth_data_weight", smooth_data_weight, 0.5);
    nh.param<double>("smooth_distance_weight", smooth_distance_weight, 0.5);
    nh.param<bool>("euclidean_distance", euclidean_distance, false);
    nh.param<bool>("diagonal_movement", diagonal_movement, false);
    


    // Transform the color_cost_string into the right format
    std::map< std::tuple<char, char, char>, double > color_cost =
        std::map< std::tuple<char, char, char>, double > ();
    if( !color_cost_string.empty() ){
        std::string line;
        std::stringstream  raw_data;
        raw_data << color_cost_string;
        while(std::getline(raw_data,line,';')){
            std::string chunck;
            std::stringstream line_data;
            line_data << line;
            std::vector<double> values;
            while(std::getline(line_data,chunck,',')){
                values.push_back(std::stod(chunck));
            }
            color_cost.insert(
                std::make_pair(std::make_tuple((char)values[0],(char)values[1],(char)values[2]),values[3]));
        }
    }


    // Create the occupancy graph
    astar_tutorial::OccupancyGraph occupancy_graph = astar_tutorial::OccupancyGraph(
        ros::package::getPath("astar_tutorial")+"/data/"+map_file_name,
        map_width,
        cell_size,
        wall_clearance,
        color_cost,
        diagonal_movement
        );

    
    // Create the AStar path finder and run it
    astar_tutorial::AStar astar = astar_tutorial::AStar(occupancy_graph, astar_lambda, euclidean_distance);
    astar_tutorial::RealCoordinates end;
    end.x = end_x;
    end.y = end_y;
    astar_tutorial::RealCoordinates start;
    start.x = start_x;
    start.y = start_y;
    std::vector<astar_tutorial::RealCoordinates > astar_path = astar.FindPath(start, end);

    // If a path have been found
    if(!astar_path.empty()){
        // Smooth the path
        astar_path = SmoothPath(astar_path, smooth_data_weight, smooth_distance_weight, smooth_tolerance);

        // Then publish it for visualisation
        nav_msgs::Path path_msg = PathToNavMsgPath(astar_path);
        ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("plan", 1);
        while(ros::ok()){
            path_publisher.publish(path_msg);
            ros::spinOnce();
        }
    }
    return 0;

}




/* Transform the path in argument into ros publishable message */
nav_msgs::Path PathToNavMsgPath(std::vector<astar_tutorial::RealCoordinates > path){
    nav_msgs::Path output;
    std::string map_id = "/map";
    ros::Time path_time = ros::Time::now();
    output.header.frame_id = map_id;
    output.header.stamp = path_time;
    for(int i = 0; i < path.size(); ++i) {
        geometry_msgs::PoseStamped p;
        p.header.stamp = path_time;
        p.header.frame_id = map_id;

        p.pose.position.x = path[i].x;
        p.pose.position.y = path[i].y;
        p.pose.position.z = 0.0;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
        output.poses.push_back(p);
    }
    return output;
}




/* Smooth the path using gradient decent */
std::vector<astar_tutorial::RealCoordinates > SmoothPath(std::vector<astar_tutorial::RealCoordinates > path,
        double data_weight, double smooth_weigth, double tolerance){

    // Initialise the smooth path with the sharp one
    std::vector<astar_tutorial::RealCoordinates > smoothed_path =
        std::vector<astar_tutorial::RealCoordinates >(path);

    ////////////////////// Your code here
    
    double sum = 0;
    double ix = 0;
    double iy = 0;
    double nx = 0;
    double ny = 0; 
    
    while (sum < tolerance)
    {
    for (int i = 1; i < path.size() - 1; ++i) {
		ix = smoothed_path[i].x;
		
		smoothed_path[i].x = smoothed_path[i].x \
		    - (data_weight + 2 * smooth_weigth) * smoothed_path[i].x \ 
		    + data_weight * path[i].x \
		    + smooth_weigth * smoothed_path[i-1].x \
		    + smooth_weigth * smoothed_path[i-1].x;
		
		nx = smoothed_path[i].x;
		
		iy = smoothed_path[i].y;
		
		smoothed_path[i].y = smoothed_path[i].y \
		    - (data_weight + 2 * smooth_weigth) * smoothed_path[i].y \
		    + data_weight * path[i].y \
		    + smooth_weigth * smoothed_path[i-1].y \
		    + smooth_weigth * smoothed_path[i+1].y;
		
		ny = smoothed_path[i].y;
		
		sum = sum + std::pow((nx - ix),2) + pow((ny - iy),2);
	}
    }
    
    
  
    //////////////////////


    return smoothed_path;
}




