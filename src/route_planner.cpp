#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    //model.FindClosestNode(start_x,end_x);
    //auto closest_start = m_Model.FindClosestNode(start_x, start_y);
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    //auto closest_end = m_Model.FindClosestNode(end_x, end_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

    //std::cout<<"Closest start : " << closest_start.x;
    //std::cout<<" Closest start : " << closest_start.y;
    //std::cout<<"Closest end : " << closest_end;
}


// TODO 3: Implement the CalculateHValue method.
// Tips:-
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    std::cout << "distance : " << node->x;
    std::cout << this->end_node->x - node->x;
    //return(abs(x2 - x1) + abs(y2 - y1));
    return node->distance(*(this->end_node));
    //RoutePlanner.closest;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->RouteModel::Node::FindNeighbors();
    std::cout << "Current node neighbors" << current_node->neighbors.size();
    for(RouteModel::Node *neighbor : current_node->neighbors){
      neighbor->parent = current_node;
      neighbor->g_value = current_node->g_value+current_node->distance(*neighbor);
      neighbor->h_value = this->CalculateHValue(neighbor);
      this->open_list.push_back(neighbor);
      neighbor->visited = true;
    }
}



bool RoutePlanner::compareGHValues(const RouteModel::Node* anode, const RouteModel::Node* bnode){
  return (anode->g_value + anode->h_value > bnode->g_value + bnode->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(this->open_list.begin(), this->open_list.end(), RoutePlanner::compareGHValues);
  //Least value f value
  RouteModel::Node* least_f_value_node = this->open_list.back();
  this->open_list.pop_back();
  return least_f_value_node;
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.

// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.




// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  this->start_node->visited = true;
  this->open_list.push_back(this->start_node);
  RouteModel::Node *current_node = nullptr;
  while (!open_list.empty())
  {
    current_node=this->NextNode();
    if(current_node->distance(*this->end_node) == 0){
      m_Model.path=this->ConstructFinalPath(current_node);
      return; // leave the A* search
    } 
    else {
      AddNeighbors(current_node); // continue searching in A*
    }
  }
    // TODO: Implement your solution here.

}
