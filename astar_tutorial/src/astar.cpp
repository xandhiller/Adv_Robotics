#include "astar.h"
#include <stdlib.h>
#include <tuple>
#include <ros/console.h>
#include <math.h>




/* Constructor : occupancy_graph the graph/map, lambda the weight of the heuristic
 * and euclidean_distance a flag to use the euclidean or manhattan distance for the heuristic*/
astar_tutorial::AStar::AStar (astar_tutorial::OccupancyGraph occupancy_graph, double lambda,
        bool euclidean_distance) {
    graph_ = occupancy_graph;
    lambda_ = lambda;
    start_id_ = 0;
    end_id_ = 0;
    euclidean_distance_ = euclidean_distance;
}
/* Destructor */
astar_tutorial::AStar::~AStar () { }







/* Compute the path between the start and end coordinates arguments with the A* path planning algorithm,
 * return a vector of RealCoordinates : x,y */
std::vector<astar_tutorial::RealCoordinates > astar_tutorial::AStar::FindPath (
         astar_tutorial::RealCoordinates start, astar_tutorial::RealCoordinates end)
{
    // Initialise the output (the node id path) of this method
    std::vector<astar_tutorial::RealCoordinates> output = std::vector<astar_tutorial::RealCoordinates>();

    // Convert the real world coordinates in grid coodrinates and in node ID,
    // and check if the nodes are part of the graph
    start_id_ = graph_.PositionToID(start);
    end_id_ = graph_.PositionToID(end);
    if( (start_id_ == -1) || (end_id_ == -1) ){
        ROS_WARN("A path can not be searched as the starting point and/or the end point is/are not in the graph");
        return output;
    }
    grid_end_ = graph_.IDToGrid(end_id_);


    // Instantiate the OpenList and ExtendedList objects
    extended_list_ = astar_tutorial::ExtendedList();
    open_list_ = astar_tutorial::OpenList(lambda_);
    
    // Push the starting node in the open list
    astar_tutorial::Node first_node;
    first_node.id = start_id_;
    first_node.parent_id = start_id_;
    first_node.cost = 0;
    first_node.heuristic = ComputeHeuristic(start_id_);
    open_list_.Push(first_node);




    //////////////////// Your code here
    
    
    
    
    
    
    
    
    ////////////////////

    ROS_INFO("Number of node extended = %d", extended_list_.Size());

    return output;
}


/* Conmpute the heuristic for a given node id */
double astar_tutorial::AStar::ComputeHeuristic(int id){
    
    astar_tutorial::GridCoordinates grid_pos = graph_.IDToGrid(id);
   
    double distance = 0; 

    if(!euclidean_distance_){
        //////////////////// Your code here (Manhattan distance).
        
        
        ////////////////////
    }else{
        //////////////////// Your code here (Euclidean distance).
        
        
        ////////////////////
    }
    return distance;
}



