#ifndef RRT_STAR_GLOBAL_PLANNER_NODE_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_NODE_HPP_

#include <cmath>
#include <vector>

namespace rrt_star_global_planner {

inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
  return std::hypot((x1 - x2), (y1 - y2));
}

struct Node {
  float x;
  float y;
  float yaw;
  std::vector<float> segmentpath_x;
  std::vector<float> segmentpath_y;
  std::vector<float> segmentpath_yaw;
  int node_id;
  int parent_id;
  float cost = 0.0;

  Node() {}

  Node(float px, float py, float pyaw, int node_index, int parent_index) : x(px),
                                                       y(py),
                                                       yaw(pyaw),
                                                       node_id(node_index),
                                                       parent_id(parent_index) {}

  bool operator ==(const Node& node) { return node_id == node.node_id; }

  bool operator !=(const Node& node) { return !(node_id == node.node_id); }
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_NODE_HPP_  NOLINT
