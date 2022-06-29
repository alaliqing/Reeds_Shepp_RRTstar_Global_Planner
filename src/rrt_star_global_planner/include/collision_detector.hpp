/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>

#include <utility>

#include "node.hpp"

namespace rrt_star_global_planner {

class CollisionDetector 
{
 public:
  explicit CollisionDetector(costmap_2d::Costmap2D* costmap);

  bool isThisPointCollides(float wx, float wy);

  // line between node and point
  bool isThereObstacleBetween(const Node &node, const std::pair<float, float> &point);

  // line between two nodes
  bool isThereObstacleBetween(const Node &node1, const Node &node2);

 private:
  void worldToMap(float wx, float wy, int& mx, int& my);  // NOLINT

  costmap_2d::Costmap2D* costmap_{nullptr};
  float resolution_{0.1};
  float origin_x_{0.0};
  float origin_y_{0.0};
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_COLLISION_DETECTOR_HPP_  NOLINT
