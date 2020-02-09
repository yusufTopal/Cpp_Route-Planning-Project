#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model)
{
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
  return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
  current_node->FindNeighbors();
  for (auto neighbor : current_node->neighbors)
  {
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = this->CalculateHValue(neighbor);
    this->open_list.push_back(neighbor);
    neighbor->visited = true;
  }
}

bool RoutePlanner::Compare(const RouteModel::Node *x, const RouteModel::Node *y)
{
  return x->g_value + x->h_value > y->g_value + y->h_value;
}

RouteModel::Node *RoutePlanner::NextNode()
{
  std::sort(this->open_list.begin(), this->open_list.end(), Compare);
  RouteModel::Node *lowest_Sum = this->open_list.back();
  this->open_list.pop_back();
  return lowest_Sum;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  while (current_node)
  {
    path_found.push_back(*current_node);
    if (current_node->parent != nullptr)
    {
      this->distance += current_node->distance(*current_node->parent);
    }
    current_node = current_node->parent;
  }
  std::reverse(path_found.begin(), path_found.end());
  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of
                                     // the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch()
{

  this->start_node->visited = true;
  RouteModel::Node *current_node = nullptr;
  this->open_list.push_back(this->start_node);

  while (open_list.size() > 0)
  {

    current_node = this->NextNode();
    if (current_node->distance(*this->end_node) == 0)
    {
      m_Model.path = this->ConstructFinalPath(current_node);
      break;
    }
    else
    {

      RoutePlanner::AddNeighbors(current_node);
    }
  }
}