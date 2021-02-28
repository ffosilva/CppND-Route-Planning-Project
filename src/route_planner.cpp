#include "route_planner.h"
#include <algorithm>

/**
 * Compare the F values of two cells.
 */
static bool Compare(const RouteModel::Node *a, const RouteModel::Node *b)
{
    float fA = a->h_value + a->g_value;
    float fB = b->h_value + b->g_value;

    return fA > fB;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);;

        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }

}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);

    auto current = open_list.back();
    open_list.pop_back();
    
    return current;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    if (!current_node) {
        return path_found;
    }

    RouteModel::Node *l_node = current_node;
    RouteModel::Node *r_node = l_node->parent;

    path_found.push_back(*l_node);

    while (r_node) {
        path_found.push_back(*r_node);
        distance += l_node->distance(*r_node);

        l_node = r_node;
        r_node = r_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    this->open_list.push_back(this->start_node);

    do
    {
        current_node = this->NextNode();
        AddNeighbors(current_node);
    } while (current_node != this->end_node);

    this->m_Model.path = this->ConstructFinalPath(current_node);
}