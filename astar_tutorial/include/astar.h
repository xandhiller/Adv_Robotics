
#ifndef ASTAR_H
#define ASTAR_H

#include <string>
#include "extended_list.h"
#include "open_list.h"
#include "occupancy_graph.h"
#include "structures.h"
#include <vector>

namespace astar_tutorial{
    class AStar
    {



    public:



        /* Constructor : occupancy_graph the graph/map, lambda the weight of the heuristic
         * and euclidean_distance a flag to use the euclidean or manhattan distance for the heuristic*/
        AStar (astar_tutorial::OccupancyGraph occupancy_graph, double lambda = 1, bool euclidean_distance = false);
        /* Destructor */
        virtual ~AStar ();


        /* Compute the path between the start and end coordinates arguments with the A* path planning algorithm,
         * return a vector of RealCoordinates  : x,y */
        std::vector<astar_tutorial::RealCoordinates> FindPath (
                astar_tutorial::RealCoordinates start, astar_tutorial::RealCoordinates end);


    private:

        // Stores the extended list, open list and occupancy graph
        astar_tutorial::ExtendedList extended_list_;
        astar_tutorial::OpenList open_list_;
        astar_tutorial::OccupancyGraph graph_;

        // Store the start and end node id and the end node grid coordinates
        int start_id_;
        int end_id_;
        astar_tutorial::GridCoordinates grid_end_;

        // Stores the weight of the heuristic in the Astar
        double lambda_;

        // Store the flag for using the euclidean or manhattan distance
        bool euclidean_distance_;
        
        
        /* Conmpute the heuristic for a given node id */
        double ComputeHeuristic(int id);

    };
}

#endif // ASTAR_H
