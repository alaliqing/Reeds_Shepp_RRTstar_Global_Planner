/*
  Copyright 2021 - Rafael Barreto
*/

#include "collision_detector.hpp"

namespace rrt_star_global_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap_ != nullptr) {
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }
}

bool CollisionDetector::isThisPointCollides(float wx, float wy) {
  // In case of no costmap loaded
  if (costmap_ == nullptr) 
  {
    // no collision
    return false;
  }

  int mx, my;
  worldToMap(wx, wy, mx, my);

  if ((mx < 0) || (my < 0) || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
  {
    return true;
  }

  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(costmap_->getCost(mx, my));

  // std::cout<<"The cost is: "<<cost<<std::endl;

  if (cost > 0)
  {
    // ROS_INFO("This point collides.");
    return true;
  }

  return false;
}

// bool CollisionDetector::isThereObstacleBetween(const Node &node, const std::pair<double, double> &point) 
// {
//   // In case of no costmap loaded
//   if (costmap_ == nullptr) 
//   {
//     // there is NO obstacles
//     return false;
//   }

//   float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
//   if (dist < resolution_) 
//   {
//     return (isThisPointCollides(point.first, point.second)) ? true : false;
//   } 
//   else 
//   {
//   int steps_number = static_cast<int>(floor(dist/resolution_));
//   float theta = atan2(node.y - point.second, node.x - point.first);
//   std::pair<float, float> p_n;
//   for (int n = 1; n < steps_number; n++) 
//   {
//     p_n.first = node.x + n*resolution_*cos(theta);
//     p_n.second = node.y + n*resolution_*sin(theta);
//     if (isThisPointCollides(p_n.first, p_n.second))
//     {
//       ROS_INFO("There is an obstacle between two nodes.");
//       return true;
//     }
//   }
//   return false;
//   }
// }

bool CollisionDetector::isThereObstacleBetween(const Node &node1, const Node &node2) 
{
  if (costmap_ == nullptr) 
    return false;

  if (isThisPointCollides(node2.x, node2.y))
    return true;
  
  if (isThisPointCollides(node1.x, node1.y))
    return true;

  int step_number = static_cast<int>(floor(0.11/(resolution_/2)));
  float theta;
  std::pair<float, float> p_n;
  for (int n = 0; n < node2.segmentpath_x.size()-1; n++) 
  {
    for (int i = 1; i < step_number; i++)
    {
      theta = atan2(node2.segmentpath_y[n] - node2.segmentpath_y[n + 1], node2.segmentpath_x[n] - node2.segmentpath_x[n + 1]);
      p_n.first = node2.segmentpath_x[n] + i * resolution_  / 2 * cos(theta);
      p_n.second = node2.segmentpath_y[n] + i * resolution_ / 2 * sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second))
      {
        return true;
      }
    }
  }
  return false;
}

void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my) {
  if (costmap_ != nullptr) {
    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;
  }
}

}  // namespace rrt_star_global_planner
