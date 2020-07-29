#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
  // RouteModel object named as m_Model's FindClosestNode function is used to get closest Nodes.
  RouteModel::Node* Cls_Node_strt=&m_Model.FindClosestNode(start_x, start_y);
  RouteModel::Node* Cls_Node_end=&m_Model.FindClosestNode(end_x, end_y);
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  //start_node and end_node(Private Member of RoutePlanner Class) is pointer to the closest Node to starting and ending Coordinates.
  	start_node = Cls_Node_strt;
    end_node = Cls_Node_end;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  //heuristic function is simply is the distance of current node to the end node. 
  	RouteModel::Node endNode=*end_node;
  	//Current Node's distacnce function is urilized to calculate the Hvalue.
    float Hvalue = node->distance(endNode);
    return Hvalue;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
 // Current_node's(Object of Node Class) FindNeighors is utilized to get all peers of current node.
 // It sets the internal variable named as neighors.
    current_node->FindNeighbors();
 //foreach loop is used to get indivdual element from <std::vector<Node *> neighbors;> public member of subclass(Node) of route_model.
    for (RouteModel::Node* peer : current_node->neighbors) {
        //Set current Node as perant Node of retrived Neighbors(Node)
        peer->parent = current_node;
      //neighbor g value is the summation of the currentNode g and distance of current node to the respective neighbor
        peer->g_value = current_node->g_value + current_node->distance(*peer);
      //Calculating the heuristic value using local function as defined above.
        peer->h_value = CalculateHValue(peer);
      //Put the neighbor to the available node to move.
        open_list.emplace_back(peer);
      //Mark the neighbor as visited
        peer->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
//Helper Function for sorting the Nodes.
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
  	//First  helper function is utilised to sort the open nodes in descending order. This function is imported from algorithm library
    // Anonymous Function is defined to set the crieteria for the sorting algorithm
    std::sort(open_list.begin(), open_list.end(), [](const auto &n1, const auto &n2)
{
  // Computing the cost(g1+h1) for the Node 1.
    float f1 = n1->g_value + n1->h_value;
  // Computing the cost(g2+h2) for the Node 2.
    float f2 = n2->g_value + n2->h_value;
    return f1 > f2;
});
    // Since the node are arrange in descending order so assign the last entry as it is the smallest.
  	RouteModel::Node* lowestMargin = open_list.back();
    // Remove the above node from the open list;
    open_list.pop_back();
    return lowestMargin;
    //RouteModel::Node *lowest_sum = open_list[0];
    // Removing the node at the first location in open_list
    //std::vector<RouteModel::Node *>::iterator it{0};
    //open_list.erase(it);
    //return lowest_sum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:

// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    // Initialize the distance with zero value
    distance = 0.0f;
    std::vector<RouteModel::Node> pathWay;

    // TODO: Implement your solution here.
    //Trace down the path by tracing back through the parents relationship of current node until reach to the start node
    while (current_node->parent) {
      //put the current node to the vector of found path.
        pathWay.push_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        // Accuamulated distance for whole the tracked pathWay
        distance += current_node->distance(parent);
        //Change the status of current node to refer it to previous node parent.
        current_node = current_node->parent;
    }
    // Push back the starting node, which has no parent thus left out from the while loop
    pathWay.push_back(*current_node);
    //To get actual measurement of diatnce between start and end point, a scaling factor(Metrics) Multiplied.	
    distance *= m_Model.MetricScale(); 
    //Reverse the order start to end point from index 0 to length of vector.
    reverse(pathWay.begin(),pathWay.end());
    return pathWay;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //A* Search at first label the start node as visited
    start_node->visited = true;
    //Push the start node to the end of open list(Available places)
    open_list.emplace_back(start_node);

    // TODO: Implement your solution here.
    //Search until the all possible option availed either we get the end point or the path is not possible
    while (!open_list.empty()) {
       
      //NextNode function first of all sort the openlist in ascending order and then return the node with lowest cost
       RouteModel::Node* current_node  = NextNode();
       
        // Loop will be terminated if the we reach the end location in our searching task. 
        if (current_node->distance(*end_node)==0){
          //Get vector of nodes that represent the path from start point to end point.
          m_Model.path = ConstructFinalPath(end_node);
            return; 
        }
        // All neighour relevant to updated current node is  added to open list.
        AddNeighbors(current_node);
               
    }
}