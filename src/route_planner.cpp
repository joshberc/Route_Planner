#include "route_planner.h"
#include <algorithm>

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node->FindNeighbors();

    for(auto& current_neighbour : current_node->neighbors){
        if(current_neighbour->visited == false){
            current_neighbour->parent = current_node;
            current_neighbour->h_value = CalculateHValue(current_neighbour);
            current_neighbour->g_value = current_node->distance(*current_neighbour) + current_node->g_value;
            current_neighbour->visited = true;
            open_list.push_back(current_neighbour);
        }
    }
    
}


bool compareHG(RouteModel::Node *currentNode, RouteModel::Node *nextNode){
    return (currentNode->h_value + currentNode->g_value) > (nextNode->h_value + nextNode->g_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    
    //Sorts List in Descending order. Largest at the front.
    sort(open_list.begin(), open_list.end(),compareHG);

    //Creates poiter to smallest node then pops it and returns the pointer.
    RouteModel::Node *temp = open_list.back();
    open_list.pop_back();
  
    return temp;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    vector<RouteModel::Node> path_found;
    

    while (current_node != start_node)
    {
        //Calculates distance
        distance += current_node->parent->distance(*current_node);

        //Addes current node to vector and sets the iterator to this nodes parent.
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    //Adds final node to vector
    path_found.emplace_back(*current_node);

    //Flips vector to correct order. Found function at "cplusplus/reference/algorithm/reverse".
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;


}

void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = start_node;
    current_node->visited = true;

    //Iterates through all nodes.
    while(current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }

    m_Model.path = ConstructFinalPath(current_node);
}