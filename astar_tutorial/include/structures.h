#ifndef STRUCTURES_H
#define STRUCTURES_H

namespace astar_tutorial{

    struct RealCoordinates{
        double x = -1;
        double y = -1;
    };

    struct GridCoordinates{
        int row = -1;
        int col = -1;
    };


    struct Node{
        // ID of the node
        int id = -1;

        // ID of the parent node
        int parent_id = -1;

        // Cost to move from the parent node to that node
        double mvt_cost = -1;

        // Cost to move from the starting node to that node
        double cost = -1;

        // Heuristic value associated to that node
        double heuristic = -1; 
    };


}

#endif
