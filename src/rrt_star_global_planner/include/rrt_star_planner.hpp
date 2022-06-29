/*
  Copyright 2021 - Rafael Barreto
*/

#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <memory>

#include "node.hpp"
#include "rrt_star.hpp"
#include "reeds_shepp.hpp"
#include "random_float_generator.hpp"

namespace rrt_star_global_planner 
{

/**
 * @class RRTStarPlanner
 * @brief Provides a ROS rrt* global planner plugin
 */
class RRTStarPlanner : public nav_core::BaseGlobalPlanner 
{
 public:
  RRTStarPlanner();

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructor for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use
   * @param  global_frame The global frame of the costmap
   */
  RRTStarPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the RRTStarPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use for planning
   * @param  global_frame The global frame of the costmap
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT

  void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,  // NOLINT
                        const std::vector<std::vector<float>> &path);

 private:
  costmap_2d::Costmap2D* costmap_{nullptr};
  bool initialized_{false};
  float map_width_;
  float map_height_;
  double radius_;
  double goal_tolerance_;
  int max_iter_;
  bool search_specific_area_{false}; //default value: true
  std::string global_frame_;
  std::shared_ptr<RRTStar> planner_;

  ros::Publisher plan_pub_;
};

}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RRT_STAR_PLANNER_HPP_  // NOLINT
