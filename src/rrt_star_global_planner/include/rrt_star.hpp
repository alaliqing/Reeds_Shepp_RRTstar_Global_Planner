#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>

#include "random_float_generator.hpp"
#include "node.hpp"
#include "collision_detector.hpp"
#include "reeds_shepp.hpp"

namespace rrt_star_global_planner {

class RRTStar {
 public:
  RRTStar(const std::vector<double> &start_point,
          const std::vector<double> &goal_point,
          costmap_2d::Costmap2D* costmap,
          double goal_tolerance,
          double radius,
          float map_width,
          float map_height,
          int max_iter);

  /**
   * @brief compute the RRT* path planning
   * @param path list of planar positions (x, y)
   * @return true if a path is found, false otherwise
   */
  bool pathPlanning(std::vector<std::vector<float>> &path);  // NOLINT

  /**
   * @brief compute random points
   * @return random planar position (x, y)
   */
  Node sampleFree();

  /**
   * @brief Get the Index of the nearest node around the new random point
   * @param point random pointed sampled
   * @return the nearest node Index
   * @note exposed for testing purposes
   */
  int getNearestNodeId(Node point);

  /**
   * @brief Create a new node
   * @param x cartesian coordinate of the new node
   * @param y cartesian coordinate of the new node
   * @param node_nearest_id
   * @note exposed for testing purposes
   */
  void createNewNode(Node& new_node);

  /**
   * @brief RRT* near neighbour search step. Selection of the parent node with lowest cost
   * @param node_nearest_id index of the nearest node around the new node
   * @note exposed for testing purposes
   */
  void chooseParent(Node& p_new);

  /**
   * @brief RRT* rewiring step. Search the best parent inside of the circular area of the new node
   * @note exposed for testing purposes
   */
  void rewire(Node p_new);

  void propagate_cost_to_leaves(Node p_new);

  Node steer(Node x1, Node x2);

  float calc_new_node_cost(Node x1, Node x2);

  void try_goal_path(Node new_node, Node goal_node);

  std::vector<Node> getNodes() const;
  
  int search_best_goal_node();

  void computeFinalPath(std::vector<std::vector<float>> &path, int last_index);  // NOLINT

  bool isGoalReached(const std::vector<float> &p_new);

 private:
  std::vector<double> start_point_;
  std::vector<double> goal_point_;
  costmap_2d::Costmap2D* costmap_{nullptr};
  std::vector<Node> nodes_;
  RandomTripleGenerator random_float_;

  bool goal_node_founded_ = false;
  int node_count_{1};
  float map_width_;
  float map_height_;
  double radius_;
  double goal_tolerance_;
  int max_iter_;
  int available_node{0};
  Node goal_node_;

  CollisionDetector cd_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_HPP_  NOLINT
