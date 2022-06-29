#ifndef RRT_STAR_GLOBAL_PLANNER_RANDOM_FLOAT_GENERATOR_HPP_  // NOLINT
#define RRT_STAR_GLOBAL_PLANNER_RANDOM_FLOAT_GENERATOR_HPP_

#include <random>
#include <cfloat>  // DBL_MAX

namespace rrt_star_global_planner {

// TODO(Rafael) allow different ranges of x and y for non square maps

class RandomTripleGenerator
{
private:
  std::random_device rd_;
  float min_value_{-1.0};
  float max_value_{1.0};

public:
  RandomTripleGenerator() = default;

  void setRange(float min, float max);

  float generate();
  float generate_angle();
};
}  // namespace rrt_star_global_planner

#endif  // RRT_STAR_GLOBAL_PLANNER_RANDOM_FLOAT_GENERATOR_HPP_  // NOLINT
