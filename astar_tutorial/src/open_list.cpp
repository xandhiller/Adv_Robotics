#include "open_list.h"
#include <ros/console.h>




/* Constructeur with lambda the weight of the heuristic */
astar_tutorial::OpenList::OpenList (double lambda) {
    lambda_ = lambda;
    node_list_ = std::map< int, astar_tutorial::Node >();
}

/* Destructor */
astar_tutorial::OpenList::~OpenList () { }







/* Pull out of the open list the node next node to extend,
 * returns a Node with at least the following fields :
 * id, parent_id cost and the heuristic */
astar_tutorial::Node astar_tutorial::OpenList::PullToExpand ()
{
    astar_tutorial::Node return_node;
    if(node_list_.empty()){
        ROS_WARN(" The OpenList is empty, no node can be pulled for expansion");
        return return_node;
    }


    
    
    //////////////////// Your code here
    
    
    
    ////////////////////
   
    return return_node;
}






/* Push a node in the open list, returns -1 if an issue occured */
int astar_tutorial::OpenList::Push(astar_tutorial::Node node) 
{
    if(node_list_.count(node.id) == 0){ 
        node_list_.insert(std::pair< int, astar_tutorial::Node >(node.id, node) );
        return 1;
    }else{
        ROS_WARN("The node %d is already in the OpenList", node.id);
        return -1;
    }
}







/* Pull a node out of the open list, returns a Node with at least the following fields :
 * id, parent_id cost and the heuristic */
astar_tutorial::Node astar_tutorial::OpenList::Pull (int id)
{
    astar_tutorial::Node output;
    if(node_list_.count(id) != 0){ 
        output = node_list_[id];
        node_list_.erase(id);
    }else{
        ROS_WARN("There is currently no node with the ID %d in the OpenList", id);
    }
    return output;
}





/* Retruns true if the node id is in the open list */
bool astar_tutorial::OpenList::Contains (int id)
{
    if(node_list_.count(id) != 0){ 
        return true;
    }else{
        return false; 
    }
    
}





/* Returns true if the list is empty, false otherwise */
bool astar_tutorial::OpenList::Empty ()
{
    return node_list_.empty();
}







/* Print the open list for debug purpose */
void astar_tutorial::OpenList::Print(){
    std::cout << std::endl << std::endl << std::endl << "********************************";
    std::cout << "OpenList with lambda = " << lambda_ << std::endl;
    if(node_list_.empty()){
        std::cout << "EMPTY" << std::endl;
    }else{
        for(std::map< int, astar_tutorial::Node >::iterator it = node_list_.begin();
                it != node_list_.end(); ++it){
            astar_tutorial::Node node = it->second;
            
            std::cout << "Node " << node.id << " : parent node = " << node.parent_id
                << " cost = " << node.cost << " and heuristic = " << node.heuristic << std::endl;
        }
    }
    std::cout << "********************************" << std::endl << std::endl;
}
