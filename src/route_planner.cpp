#include "route_planner.h"
#include <algorithm>

/*
I have used the base code that was provided by Udacity
Researched the material provided and also performed some online google queries about A* algorithm and its implementation
I have developed my code afterwards*/

/* Although there were some errors initially, I was able to fix the errors and perform an error free compile and execution*/

/*Code for the Route planner function*/
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    std::cout << "distance : " << node->x << "  ";
    std::cout << this->end_node->x - node->x  << "\n";
    return node->distance(*(this->end_node));
}

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->RouteModel::Node::FindNeighbors();
    //std::cout << "Current node neighbors" << current_node->neighbors.size();
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

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  std::sort(this->open_list.begin(), this->open_list.end(), RoutePlanner::compareGHValues);
  //Least value f value
  RouteModel::Node* least_f_value_node = this->open_list.back();
  this->open_list.pop_back();
  return least_f_value_node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {

    std::vector<RouteModel::Node> path_found;
    // keep looping until it finds the start node, whose parent is nullptr
    RouteModel::Node* curNode = current_node;
    while (curNode) {
        path_found.push_back(*curNode);
        // Also keep track of the total path distance
        if (curNode->parent) this->distance += curNode->distance(*curNode->parent); 
        curNode = curNode->parent;
    }

    // Scale by multiplying by the model's scale
    this->distance *= m_Model.MetricScale();

    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
/* A* Implementation code*/
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
}
