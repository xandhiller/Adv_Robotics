
#ifndef OPENLIST_H
#define OPENLIST_H

#include <string>
#include <map>
#include <tuple>
#include "structures.h"

namespace astar_tutorial{
    class OpenList
    {
    public:

        /* Constructeur with lambda the weight of the heuristic */
        OpenList (double lambda = 1);

        /* Destructor */
        virtual ~OpenList ();



        /* Push a node in the open list, returns -1 if an issue occured 
         * The argument is a Node with at least the following fields :
         * id, parent_id cost and the heuristic */
        int Push (astar_tutorial::Node new_node);

        /* Pull a node out of the open list, returns a Node with at 
         *  least the following fields :
         * id, parent_id cost and the heuristic */
        astar_tutorial::Node Pull (int id);

        /* Retruns true if the node id is in the open list */
        bool Contains (int id);

        /* Returns true if the list is empty, false otherwise */
        bool Empty ();

        /* Pull out of the open list the node next node to extend, 
         * returns a Node with at least the following fields :
         * id, parent_id cost and the heuristic */
        astar_tutorial::Node PullToExpand ();

        /* Print the open list for debug purpose */
        void Print();


    private:
        // Stores the node list : the map key is the node id
        // the value is a Node with at least the following fields :
        // id, parent_id cost and the heuristic
        std::map< int, astar_tutorial::Node >node_list_;

        // Stores the heuristic weight for choosing which node to expand 
        double lambda_;


    };
}

#endif // OPENLIST_H
