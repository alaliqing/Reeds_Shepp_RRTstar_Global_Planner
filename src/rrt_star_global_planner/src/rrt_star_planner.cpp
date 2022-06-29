#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rrt_star_planner.hpp"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_global_planner::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star_global_planner 
{

nav_msgs::Path rviz_path;

RRTStarPlanner::RRTStarPlanner() : costmap_(nullptr), initialized_(false) {}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) : costmap_(nullptr), initialized_(false) 
{
  // initialize the planner
  initialize(name, costmap_ros);
}

RRTStarPlanner::RRTStarPlanner(std::string name,
                               costmap_2d::Costmap2D* costmap,
                               std::string global_frame) : costmap_(nullptr), initialized_(false) 
{
  // initialize the planner
  initialize(name, costmap, global_frame);
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) 
{
  if (!initialized_) 
  {
    costmap_ = costmap;
    global_frame_ = global_frame;

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("radius", radius_, 1.0);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.5);
    private_nh.param("max_iter", max_iter_, 500);

    if (search_specific_area_) 
    {
      map_width_ = 10.0;
      map_height_ = 10.0;
    } else 
    {
      map_width_ = costmap_->getSizeInMetersX();
      map_height_ = costmap_->getSizeInMetersY();
    }

    ROS_INFO("RRT* Reedsshepp Global Planner initialized successfully.");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    initialized_ = true;
  } 
  else 
  {
    ROS_WARN("This planner has already been initialized... doing nothing.");
  }
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) 
{
  plan.clear();
  rviz_path.poses.clear();

  ROS_INFO("RRT* Reedsshepp Global Planner");
  ROS_INFO("Current Position: ( %.2lf, %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));
  ROS_INFO("GOAL Position: ( %.2lf, %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));

  std::vector<double> start_point = {start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation)};
  std::vector<double> goal_point = {goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation)};

  planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                  goal_point,
                                                  costmap_,
                                                  goal_tolerance_,
                                                  radius_,
                                                  map_width_,
                                                  map_height_,
                                                  max_iter_));
  std::vector<std::vector<float>> path;

  ros::Time plan_time = ros::Time::now();
  //rviz_path.poses.resize(plan.size());
  rviz_path.header.frame_id = global_frame_;
  rviz_path.header.stamp = plan_time;

  if (planner_->pathPlanning(path)) 
  {
    ROS_INFO("RRT* Reedsshepp Global Planner: Path found!!!!");
    computeFinalPlan(plan, path);
    plan_pub_.publish(rviz_path);
    return true;
  } 
  else 
  {
    ROS_WARN("The planner failed to find a path, choose other goal position");
    return false;
  }
}

void RRTStarPlanner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan,
                                       const std::vector<std::vector<float>> &path) 
{
  // clean plan
  plan.clear();
  ros::Time plan_time = ros::Time::now();

  // convert points to poses
  for (int i = 0; i < path.size() - 1; i++) 
  {
    geometry_msgs::PoseStamped pose;
    tf2::Quaternion point_Quaternion;
    point_Quaternion.setRPY(0, 0, path[i][2]);
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = path[i][0];
    pose.pose.position.y = path[i][1];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(point_Quaternion);
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
    rviz_path.poses.push_back(pose);
    ROS_INFO("path x %f, path y %f", pose.pose.position.x, pose.pose.position.y);
  }
}

}  // namespace rrt_star_global_planner
