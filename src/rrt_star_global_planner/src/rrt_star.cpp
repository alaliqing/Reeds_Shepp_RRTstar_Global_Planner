#include "rrt_star.hpp"
#include <boost/math/constants/constants.hpp>

namespace rrt_star_global_planner 
{
const float pi = boost::math::constants::pi<float>();
RRTStar::RRTStar(const std::vector<double> &start_point,
                 const std::vector<double> &goal_point,
                 costmap_2d::Costmap2D* costmap,
                 double goal_tolerance,
                 double radius,
                 float map_width,
                 float map_height,
                 int max_iter) : start_point_(start_point),
                                  goal_point_(goal_point),
                                  costmap_(costmap),
                                  goal_tolerance_(goal_tolerance),
                                  radius_(radius),
                                  map_width_(map_width),
                                  map_height_(map_height),
                                  max_iter_(max_iter),
                                  cd_(costmap)
                                  
{
  // Set range
  random_float_.setRange(-map_width_, map_width_);
}

bool RRTStar::pathPlanning(std::vector<std::vector<float>> &path) 
{
  if (cd_.isThisPointCollides(goal_point_[0], goal_point_[1])) 
  {
    ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
    return false;
  }

  // Start Node
  Node node_start;
  node_start.x = start_point_[0];
  node_start.y = start_point_[1];
  node_start.yaw = start_point_[2]; 
  node_start.cost = 0;
  node_start.node_id = 0;
  node_start.parent_id = -1;
  nodes_.emplace_back(node_start);

  Node goal_node;
  goal_node.x = goal_point_[0];
  goal_node.y = goal_point_[1];
  goal_node.yaw = goal_point_[2];

  Node p_rand;
  Node p_new;
  Node node_nearest;
  int last_index;

  try_goal_path(node_start, goal_node);
  if (nodes_.size() > 1)
  {
    computeFinalPath(path, 1);
    ROS_INFO("best path!");
    return true;
  }

  for (int i = 1; i < max_iter_; i++)
  {
 //   ROS_INFO("RRT* Reedsshepp iteration: %i", i);
    p_rand = sampleFree();
    node_nearest = nodes_[getNearestNodeId(p_rand)];
    p_new = steer(node_nearest, p_rand);

    if (!cd_.isThereObstacleBetween(node_nearest, p_new)) 
    {
      createNewNode(p_new);
      try_goal_path(p_new, goal_node);
    }
    if (i >= max_iter_-1 || available_node >= 100)
    {
      ROS_INFO("reached max iteration or find enough available paths");
      last_index = search_best_goal_node();
      if (last_index)
      {
        computeFinalPath(path, last_index);
        return true;
      }
    }
  }
 ROS_WARN("goal node founded %d", goal_node_founded_);
  return false;
}

float mod2pi(float theta)
{
    float v = fmod(theta, 2 * pi);
    if (v < -pi)
        v += 2 * pi;
    else
        if(v > pi)
            v -= 2 * pi;
    return v;
}

Node RRTStar::sampleFree() 
{
  Node random_point {};
  ReedsShepp RS;
  random_point.x = random_float_.generate();
  random_point.y = random_float_.generate();
  random_point.yaw = random_float_.generate_angle();
  // random_point.yaw = mod2pi(random_point.yaw);
  return random_point;
}

int RRTStar::getNearestNodeId(Node point) 
{
  float dist_nearest, dist;
  Node node_nearest = nodes_[0];
  for (int i = 1; i < nodes_.size(); i++) 
  {
    // ROS_INFO("index: %d, node id: %d", i, nodes_[i].node_id);
    dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.x, point.y);
    dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.x, point.y);
    if (dist < dist_nearest) node_nearest = nodes_[i];
  }

//  ROS_INFO("node id: %d", node_nearest.node_id);
  return node_nearest.node_id;
}

void RRTStar::createNewNode(Node& new_node) 
{
  if (new_node.parent_id != -1)
  {
    chooseParent(new_node);
    new_node.node_id = node_count_;
    nodes_.emplace_back(new_node);
    ROS_INFO("node id: %d", new_node.node_id);
    rewire(new_node);
    node_count_ = node_count_ + 1;
  }
}

void RRTStar::chooseParent(Node& p_new) 
{
  int mincost_id = p_new.parent_id;
  Node p_new_alternate = p_new;
  Node candidate_new_node;

  float min_cost = calc_new_node_cost(nodes_[p_new.parent_id], p_new);
  
  for (const auto& node : nodes_) 
  {
    float node_dist = euclideanDistance2D(node.x, node.y, p_new_alternate.x, p_new_alternate.y);
    if (node_dist < radius_)
    {
      candidate_new_node = steer(node, p_new_alternate);
      if (!cd_.isThereObstacleBetween(node, candidate_new_node))
        {
          if (calc_new_node_cost(node, candidate_new_node) < candidate_new_node.cost)
          {
            min_cost = calc_new_node_cost(node, candidate_new_node);
            mincost_id = node.node_id;
          }
        }
    }
  }
  p_new = steer(nodes_[mincost_id], p_new);
  p_new.cost = calc_new_node_cost(nodes_[mincost_id], p_new);    
 // ROS_INFO("p_new parent id, x, y, yaw: %d, %f, %f, %f", p_new.parent_id, p_new.x, p_new.y, euclideanDistance2D(p_new.x, p_new.y, nodes_[p_new.parent_id].x, nodes_[p_new.parent_id].y));
}

void RRTStar::rewire(Node p_new) 
{
  float nodes_dist;
  Node candidate_node;

  for (int i = 0; i < nodes_.size()-1; i++) 
  {
    if (nodes_[i] == nodes_[p_new.parent_id]) continue;
    nodes_dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, p_new.x, p_new.y);
    if (nodes_dist < radius_ ) 
    {
      candidate_node = steer(p_new, nodes_[i]);
      candidate_node.cost = calc_new_node_cost(p_new, nodes_[i]);
      if (candidate_node.cost < nodes_[i].cost && !cd_.isThereObstacleBetween(p_new, candidate_node)) 
      {
        nodes_[i].x = candidate_node.x;
        nodes_[i].y = candidate_node.y;
        nodes_[i].yaw = candidate_node.yaw;
        nodes_[i].segmentpath_x = candidate_node.segmentpath_x;
        nodes_[i].segmentpath_y = candidate_node.segmentpath_y;
        nodes_[i].segmentpath_yaw = candidate_node.segmentpath_yaw;
        nodes_[i].parent_id = candidate_node.parent_id;
        nodes_[i].cost = candidate_node.cost;
        // ROS_INFO("rewire node id %d, node parent id %d", nodes_[i].node_id, nodes_[i].parent_id);
        propagate_cost_to_leaves(p_new);
      }
    }
  }
}

void RRTStar::propagate_cost_to_leaves(Node p_new)
{
  for (int i = 0; i < nodes_.size()-1; i++)
  {
    if (nodes_[i].parent_id == p_new.node_id)
    {
      nodes_[i].cost = calc_new_node_cost(p_new, nodes_[i]);
      propagate_cost_to_leaves(nodes_[i]);
    }
  }
}

Node RRTStar::steer(Node x1, Node x2) 
{
  std::vector<std::vector<float>> node_result;
  std::vector<float> result_x;
  std::vector<float> result_y;
  std::vector<float> result_yaw;
  float result_length;
  Node p_new {};
  ReedsShepp RS;
  float q0[] = {x1.x, x1.y, x1.yaw};
  float q1[] = {x2.x, x2.y, x2.yaw};
  node_result = RS.ReedsPathsample(q0, q1, 0.11, result_length, result_x, result_y, result_yaw);
  // p_new.x = node_result.back()[0];
  p_new.x = node_result[(node_result.size()-1)][0];
  p_new.y = node_result[(node_result.size()-1)][1];
  p_new.yaw = node_result[(node_result.size()-1)][2];
  p_new.yaw = mod2pi(p_new.yaw);
  p_new.segmentpath_x = result_x;
  p_new.segmentpath_y = result_y;
  for (int i = 0; i < result_yaw.size()-1; i++)
    result_yaw[i] = mod2pi(result_yaw[i]);
  p_new.segmentpath_yaw = result_yaw;
  p_new.parent_id = x1.node_id;
//  ROS_INFO("NODE COST: %f", p_new.cost);

  return p_new;
}

float RRTStar::calc_new_node_cost(Node x1, Node x2)
{
  ReedsShepp RS;
  float new_node_cost;
  std::vector<std::vector<float>> node_result;
  std::vector<float> result_x;
  std::vector<float> result_y;
  std::vector<float> result_yaw;
  float result_length;
  float q0[] = {x1.x, x1.y, x1.yaw};
  float q1[] = {x2.x, x2.y, x2.yaw};
  node_result = RS.ReedsPathsample(q0, q1, 0.1, result_length, result_x, result_y, result_yaw);
  new_node_cost = result_length + x1.cost;
  return new_node_cost;
}

void RRTStar::try_goal_path(Node new_node, Node goal_node)
{
  Node new_node_2 = steer(new_node, goal_node);
  if (!cd_.isThereObstacleBetween(new_node, new_node_2))
  {
    new_node_2.node_id = node_count_;
    new_node_2.cost = calc_new_node_cost (new_node, goal_node);
    available_node++;
    ROS_INFO("available path: %d, node count %d", available_node, node_count_);
    nodes_.emplace_back(new_node_2);
    node_count_ = node_count_ + 1;
  }
}

int RRTStar::search_best_goal_node()
{
  std::vector<int> goal_indexes;
  int min_cost_id;
  for (int i = 0; i < nodes_.size(); i++)
    if (euclideanDistance2D(nodes_[i].x, nodes_[i].y, goal_point_[0], goal_point_[1]) <= goal_tolerance_)
      goal_indexes.emplace_back(nodes_[i].node_id);
  
//  for (int i =0; i< goal_indexes.size(); i++)
//    ROS_INFO("goal indexes %d", goal_indexes[i]);

  std::vector<int> final_goal_indexes;
  for (int i: goal_indexes)
  {
    // ROS_INFO("angle comparision %f, %f", fabs(nodes_[i].yaw - goal_point_[2]), 0.0873);
    // ROS_INFO("distance comparision: %f, %f\n", euclideanDistance2D(nodes_[i].x, nodes_[i].y, goal_point_[0], goal_point_[1]), goal_tolerance_);
    if (fabs(nodes_[i].yaw - goal_point_[2]) <= 0.0873)
      final_goal_indexes.emplace_back(i);
  }
  
  if (final_goal_indexes.size() == 0)
    return 0;

  ROS_INFO("final indexes size: %d", final_goal_indexes.size());
  
  float min_cost = nodes_[final_goal_indexes[0]].cost;
  min_cost_id = final_goal_indexes[0];
  for (int i = 0; i < final_goal_indexes.size() - 1; i++)
  {
    //ROS_INFO("min cost %f", min_cost);
    if (nodes_[final_goal_indexes[i]].cost < min_cost)
      {
        min_cost = nodes_[final_goal_indexes[i]].cost;
        min_cost_id = final_goal_indexes[i];
      }
  }
  ROS_INFO ("final index ID: %d", min_cost_id);
  return min_cost_id;
}


void RRTStar::computeFinalPath(std::vector<std::vector<float>>& path, int last_index) 
{ 
  path.clear();
  // Compute the path from the goal to the start
  Node current_node = nodes_[last_index];
  std::vector<float> point;
  point.emplace_back(goal_node_.x);
  point.emplace_back(goal_node_.y);
  point.emplace_back(goal_node_.yaw);
  path.insert(path.begin(), point);
  do 
  {
    ROS_INFO("node id %d, node parent %d", current_node.node_id, current_node.parent_id);
    // point.first = current_node.x;
    // point.second = current_node.y;
    // path_list.push_front(point);
    for (int i = current_node.segmentpath_x.size() - 1; i >0; i--)
      {
        point.clear();
        point.emplace_back(current_node.segmentpath_x[i]);
        point.emplace_back(current_node.segmentpath_y[i]);
        point.emplace_back(current_node.segmentpath_yaw[i]);
        path.insert(path.begin(), point);
      }
    current_node = nodes_[current_node.parent_id];
  } 
  while (current_node.parent_id != -1);
}
}
