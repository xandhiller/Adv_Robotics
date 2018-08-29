
#ifndef EXTENDEDLIST_H
#define EXTENDEDLIST_H

#include <string>
#include <map>
#include <vector>
#include "structures.h"

namespace astar_tutorial{
    class ExtendedList
    {
    public:

        
        /* Constructor */
        ExtendedList ();

        /* Destructor */
        virtual ~ExtendedList ();


        /* Push a node to the Extended list, returns -1 if an issue occured
         * The argument Node need to have the following fields : id, parent_id, cost*/
        int Push (astar_tutorial::Node new_node);

        /* Get the path from the end_id node : retrun a vector of id (int) going
         * from the starting node to the ending node */
        std::vector<int> GetPath (int end_id);


        /* Return true if the node id is in the list, false otherwise */
        bool Contains (int id);

        /* Return the list size */
        int Size();

        /* Print the extended list for (debug purpose) */
        void Print();

    private:

        //  Store the node list : the map key is the node id
        //  the value is a Node with at least the following fields :
        //  id, parent_id and cost
        std::map< int, astar_tutorial::Node > node_list_;



    };
}

#endif // EXTENDEDLIST_H
