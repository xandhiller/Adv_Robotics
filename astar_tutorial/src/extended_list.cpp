#include "extended_list.h"
#include <ros/console.h>



/* Constructor */
astar_tutorial::ExtendedList::ExtendedList () {
    node_list_ = std::map< int, astar_tutorial::Node >();
}

/* Destructor */
astar_tutorial::ExtendedList::~ExtendedList () { }







/* Get the path from the end_id node : return a vector of id (int) going from the starting node to the ending node */
std::vector<int> astar_tutorial::ExtendedList::GetPath (int end_id)
{
    std::vector<int> output = std::vector<int>();

    //////////////////// Your code here
    
	int current_id = end_id;

    while(!(node_list_[current_id].cost == 0)) {
	    output.push_back(current_id);
	    current_id = node_list_[current_id].parent_id;
	}

	std::reverse(output.begin(),output.end());
    ////////////////////
    

    return output;
}




/* Push a node to the Extended list, return -1 if an issue occured */
int astar_tutorial::ExtendedList::Push (astar_tutorial::Node new_node)
{
    if( (node_list_.empty() ) ||
            ( (node_list_.count(new_node.id) == 0) && (node_list_.count(new_node.parent_id)!=0) ) ){ 
        node_list_.insert(std::pair< int, astar_tutorial::Node >(
                    new_node.id, new_node));
        return 1;
    }else{
        ROS_WARN("The node %d is already in the ExtendedList and/or the parent node %d is not in the ExtendedList", new_node.id, new_node.parent_id);
        return -1;
    }
}





/* Return the list size */
int astar_tutorial::ExtendedList::Size(){
    return node_list_.size();
}






/* Return true if the node id is in the list, false otherwise */
bool astar_tutorial::ExtendedList::Contains (int id)
{
    if(node_list_.count(id) != 0){
        return true;
    }else{
        return false;
    }
}






/* Print the extended list for (debug purpose) */
void astar_tutorial::ExtendedList::Print(){
    std::cout << std::endl << std::endl << std::endl << "********************************";
    std::cout << "ExtendedList" << std::endl;
    if(node_list_.empty()){
        std::cout << "EMPTY" << std::endl;
    }else{
        for(std::map<int, astar_tutorial::Node >::iterator it = node_list_.begin();
                it != node_list_.end(); ++it){
            int id = it->first;
            astar_tutorial::Node node = it->second;

            std::cout << "Node " << node.id << " : cost = " << node.cost 
                << " and parent node  = " << node.parent_id << std::endl;
        }
    }
    std::cout << "********************************" << std::endl << std::endl;
}

