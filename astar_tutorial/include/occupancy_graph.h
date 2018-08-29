
#ifndef OCCUPANCYGRAPH_H
#define OCCUPANCYGRAPH_H

#include <tuple>
#include <string>
#include <vector>
#include <map>
#include "structures.h"

namespace astar_tutorial{
    class OccupancyGraph
    {
    public:


        /* Empty Constructor */
        OccupancyGraph ();

        /* Constructor 
         * obstacle_file_path is the map image path
         * grid_resolution is the size in meter of a cell (cells are square)
         * clearance is the clearance to the walls (configuration space)
         * color_cost is the collection of the colors used in the map with there associated cost
         * diagonal_movement is a flag to allow or not the diagnal movement is the grid*/
        OccupancyGraph (std::string obstacle_file_path,
                        double width,
                        double grid_resolution,
                        double clearance = 0.0,
                        std::map< std::tuple<char, char, char>, double > color_cost =
                            std::map< std::tuple<char, char, char>, double >(),
                        bool diagonal_movement = false
                            );

        /* Destructor */
        virtual ~OccupancyGraph ();





    private:

        
        // Stores the graph in a vector of node, the postion of the node
        // in the vector (index) is the node id
        // a node is a tuple : grid_row, grid_col, real_y, real_x, cost
        std::vector< std::tuple<int, int, double, double, double> > node_list_;

        // Grid size in cells
        int grid_height_;
        int grid_width_;

        // Map size
        double real_height_;
        double real_width_;

        // Resolutions
        // Meter per pixel
        double pixel_resolution_;
        // Meter per cell
        double grid_resolution_;

        // Number of nodes in the graph
        int nb_nodes_;

        // Stores the correpondence between an node coordinates in the
        // occupancy grid and its ID (this storage prevent and extensive
        // 'reverse search in node_list_
        // The key of the map is the node grid coordinates
        // the value is the node ID
        std::map< std::pair<int, int>, int > grid_to_id_;

        // Are diagonal movements allowed (yes if true)
        bool diagonal_movement_;

        // Stores the color-cost correpondances
        std::map< std::tuple<char, char, char>, double> color_cost_;

        /* Give the cost of movement associated to the colot */
        double ColorToCost (char r, char g, char b){
            if(color_cost_.count(std::make_tuple(r, g, b))!=0){
                return color_cost_[std::make_tuple(r, g, b)];
            }else{
                return 1;
            }
		}



    public:


        /* Returns the grid coordinates corresponding to the node id 
         * The coordinates are represented with a GridCoordinates : row, column*/
        astar_tutorial::GridCoordinates IDToGrid(int id);
        
        /* Returns the real (meters) coordinates corresponding to the node id 
         * The coordinates are represented with a pair : x, y*/
        astar_tutorial::RealCoordinates IDToPosition(int id);
        
        /* Get adjacent nodes in the graph to the node id,
         * retruns a vector of Node, only the fields id and mvt_cost are completed */
        std::vector<astar_tutorial::Node > GetAdjacentNodes(int node_ID);

        /* Returns the node id correponding to a real (meters) point coordinates */
        int PositionToID(astar_tutorial::RealCoordinates pos);

    };
}

#endif // OCCUPANCYGRAPH_H
