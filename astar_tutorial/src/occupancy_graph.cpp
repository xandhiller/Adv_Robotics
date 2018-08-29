#include "occupancy_graph.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/console.h>


/* Empty Constructor */
astar_tutorial::OccupancyGraph::OccupancyGraph(){
}

/* Constructor
 * obstacle_file_path is the map image path
 * grid_resolution is the size in meter of a cell (cells are square)
 * clearance is the clearance to the walls (configuration space)
 * color_cost is the collection of the colors used in the map with there associated cost
 * diagonal_movement is a flag to allow or not the diagnal movement is the grid*/
astar_tutorial::OccupancyGraph::OccupancyGraph(
        std::string obstacle_file_path,
        double width,
        double grid_resolution,
        double clearance,
        std::map< std::tuple<char, char, char>, double > color_cost,
        bool diagonal_movement){
    

    color_cost_ = color_cost;
    diagonal_movement_ = diagonal_movement;
    grid_resolution_ = grid_resolution;

    // Check is all the wieght should be equal
    bool unique_weight = false;
    if(color_cost.empty()){
        unique_weight = true;
    }

    // Read the obstacle map image
    cv::Mat raw_img;
    raw_img = cv::imread(obstacle_file_path, cv::IMREAD_COLOR );

    // If there is a problem when reading the map image abort
    if(! raw_img.data )
    {
        ROS_WARN("Could not open or find the obstacle map image");
        return;
    }


    // Create an occupancy image from the color black
    cv::Mat obstacle_img;
    cv::inRange(raw_img, cv::Scalar(0, 0, 0), cv::Scalar(5, 5, 5), obstacle_img);


    // Compute the real (meters) size of the map
    real_width_ = width;
    real_height_ = width * obstacle_img.rows / obstacle_img.cols;
    



    // Treat the clearance to wall dilating the map image
    pixel_resolution_ = width / obstacle_img.cols;
    if(clearance != 0){
        int dilate_size = ceil(clearance / pixel_resolution_);
        cv::Mat structuring_element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1),
                cv::Point(dilate_size, dilate_size));

        cv::dilate(obstacle_img, obstacle_img, structuring_element);
    }


    // Compute the grid size
    grid_width_ = floor(width / grid_resolution);
    grid_height_ = floor(obstacle_img.rows * width / (grid_resolution*obstacle_img.cols));
    double cell_size = obstacle_img.cols * grid_resolution / width;

    // Initialise the different storage variables
    grid_to_id_ = std::map< std::pair<int, int>, int >();
    nb_nodes_ = 0;
    node_list_ = std::vector< std::tuple<int, int, double, double, double> >();
    node_list_.reserve(obstacle_img.cols*obstacle_img.rows);

    // For each cell of the grid map
    for(int row = 0; row<grid_height_; ++row){
        for(int col = 0; col<grid_width_; ++col){
                      
            // Compute the corresponding area in px
            int row_start = round(row*cell_size);
            int row_end = round((row+1)*cell_size)-1;
            if(row == (grid_height_-1)){
                row_end = obstacle_img.rows-1;
            }
            int col_start = round(col*cell_size);
            int col_end = round((col+1)*cell_size)-1;
            if(col == (grid_width_-1)){
                col_end = obstacle_img.cols-1;
            }

            // Get the pixel in the middle of the area for the cost
            int cost_px_row = round( (row_end + row_start)/2.0);
            int cost_px_col = round( (col_end + col_start)/2.0);


            // Compute the real coordinates in meters associated to the cell
            double real_y = (row + 0.5) * cell_size * pixel_resolution_;
            double real_x = (col + 0.5) * cell_size * pixel_resolution_;
            
            
            // Get the max intensity of the cell area in the occupancy image
            double min_cell_intensity;
            double max_cell_intensity;
            if( ( (col_end-col_start )<= 1) || ((row_end-row_start) <=1 ) ){
                max_cell_intensity = obstacle_img.at<uchar>(row_start, col_start);
            }else{
                cv::Mat cell_pixels = obstacle_img(
                        cv::Rect( col_start, row_start, (col_end-col_start ), (row_end-row_start) ) );
                cv::minMaxLoc(cell_pixels, &min_cell_intensity, &max_cell_intensity);
            }


            // If the cell is fully free add a node to the node list (with the corresponding cost
            if(max_cell_intensity == 0){
                grid_to_id_.insert(std::pair< std::pair<int, int>, int >(std::pair<int, int>(row,col), nb_nodes_));
                
                double cost = 1;
                if(!unique_weight){
                    cost = astar_tutorial::OccupancyGraph::ColorToCost(
                            raw_img.at<cv::Vec3b>(cost_px_row,cost_px_col).val[2],
                            raw_img.at<cv::Vec3b>(cost_px_row,cost_px_col).val[1],
                            raw_img.at<cv::Vec3b>(cost_px_row,cost_px_col).val[0]);
                }
                node_list_.push_back(std::make_tuple(row, col, real_height_ - real_y, real_x, cost));
                nb_nodes_++;

            }

        }
    }
    node_list_.shrink_to_fit();


}


/* Destructor */
astar_tutorial::OccupancyGraph::~OccupancyGraph () { }







/* Get adjacent nodes in the graph to the node id,
 * retruns a vector of pair, each pair represent an adjacent node : id, cost */
std::vector<astar_tutorial::Node > astar_tutorial::OccupancyGraph::GetAdjacentNodes(int id){

    std::vector<astar_tutorial::Node > neighbour_list = std::vector<astar_tutorial::Node >();
    if( (id < 0) || (id >= nb_nodes_)){
        ROS_WARN("There is no node %d in the graph", id);
        return neighbour_list;
    }
    

    int row = std::get<0>(node_list_[id]);
    int col = std::get<1>(node_list_[id]);
    double terrain_cost = std::get<4>(node_list_[id]);

    // If the robot can only move up, down, left, right
    if(!diagonal_movement_){
        const int kRowMvt[4] = {1, 0, -1, 0};
        const int kColMvt[4] = {0, -1, 0, 1};

        // For each possible movement
        for(int i = 0; i<4; ++i){
            // If node exist in the surrounding place
            if(grid_to_id_.count(std::pair<int, int>(row + kRowMvt[i], col + kColMvt[i])) != 0){
                int neighbour_id = grid_to_id_[std::pair<int, int>(row + kRowMvt[i], col + kColMvt[i])];
            
                double edge_cost = std::max(terrain_cost, std::get<4>(node_list_[neighbour_id]));
                astar_tutorial::Node node_to_push;
                node_to_push.id = neighbour_id;
                node_to_push.mvt_cost = edge_cost;
                neighbour_list.push_back(node_to_push); 
            }
        }

    // Else (the robot can also move diagonally)
    }else{
        //////////////////// Your code here
        
        
        
        
        ////////////////////
    }


    return neighbour_list;
}






/* Returns the node id correponding to a real (meters) point coordinates */
int astar_tutorial::OccupancyGraph::PositionToID(astar_tutorial::RealCoordinates pos){
    if( (pos.x < 0) || (pos.y < 0) || (pos.x >= real_width_) || (pos.y >= real_height_) ){
        ROS_WARN("The position x = %f, y = %f is out of the map", pos.x, pos.y);
        return -1;
    }
    int col = floor(pos.x / grid_resolution_);
    if(col >= grid_width_){
        col = grid_width_ - 1;
    }
    int row = floor((real_height_ - pos.y) / grid_resolution_);
    if(row >= grid_height_){
        row = grid_height_ - 1;
    }
    if(grid_to_id_.count(std::pair<int, int>(row, col)) != 0){
        return grid_to_id_[std::pair<int, int>(row, col)];
    }else{
        ROS_WARN("The cell associated to the position x = %f, y = %f is occupied", pos.x, pos.y);
        return -1;
    }


}






/* Returns the grid coordinates corresponding to the node id 
 * The coordinates are represented with a pair : row, collumn*/
astar_tutorial::GridCoordinates astar_tutorial::OccupancyGraph::IDToGrid(int id){
    astar_tutorial::GridCoordinates output;
    if( (id < 0) || (id >= nb_nodes_)){
        ROS_WARN("There is no node %d in the graph", id);
        return output;
    }
    output.row = std::get<0>(node_list_[id]);
    output.col = std::get<1>(node_list_[id]);
    return output;
}







/* Returns the real (meters) coordinates corresponding to the node id 
 * The coordinates are represented with a pair : x, y*/
astar_tutorial::RealCoordinates astar_tutorial::OccupancyGraph::IDToPosition(int id){
    astar_tutorial::RealCoordinates output;
    if( (id < 0) || (id >= nb_nodes_)){
        ROS_WARN("There is no node %d in the graph", id);
        return output;
    }
    output.x = std::get<3>(node_list_[id]);
    output.y = std::get<2>(node_list_[id]);
    return output;
}
