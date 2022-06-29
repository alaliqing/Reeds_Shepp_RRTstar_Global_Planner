#include "random_float_generator.hpp"
#include "iostream"
#include "boost/math/constants/constants.hpp"

namespace rrt_star_global_planner 
{
const float pi = boost::math::constants::pi<float>();
void RandomTripleGenerator::setRange(float min, float max) 
{
  min_value_ = min;
  max_value_ = max;
}

float RandomTripleGenerator::generate() 
{
  std::mt19937 gen(rd_());

  // Note: uniform_real_distribution does [start, stop), but we want to do [start, stop].
  // Therefore passing the next largest value instead.
  return std::uniform_real_distribution<float> {min_value_, max_value_}(gen);
}

float RandomTripleGenerator::generate_angle()
{
  std::mt19937 gen1(rd_());
  return std::uniform_real_distribution<float> {-pi, pi}(gen1);
}  // namespace rrt_star_global_planner
}

// int main()
// {
//   rrt_star_global_planner::RandomTripleGenerator RTG;
//   for (int i = 0; i < 30; i++)
//   {
//     double a = RTG.generate_angle();
//     std::cout<<a<<std::endl;
//   }
//   return 0;
// }