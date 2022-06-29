#ifndef REEDS_SHEPP_
#define REEDS_SHEPP_

#include <vector>
#include <cassert>
#include <typeinfo>
#include <boost/math/constants/constants.hpp>

class ReedsShepp
{
public:
    enum SegmentType
    {
        NOP = 0,
        LEFT = 1,
        STRAIGHT = 2,
        RIGHT = 3,
    };
    static const SegmentType PathType[18][5];
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const SegmentType* type = PathType[0],
                       float t = std::numeric_limits<float>::max(), float u = 0., float v = 0.,
                       float w = 0., float x = 0.);
        float length() const 
        {
            return totalLength_; 
        }
        const SegmentType* type_;   
        float length_[5];            
        float totalLength_;
    };
    float rho_ = 1.5;
    float distance(float q0[3], float q1[3]);
    std::vector <int> ReedsPathtype(float q0[3], float q1[3]);
    std::vector<std::vector<float> > ReedsPathsample(float q0[], float q1[], float step_size, float& result_length,
                                                            std::vector<float>& result_x, std::vector<float>& result_y, std::vector<float>& result_yaw);
    ReedsSheppPath reedsShepp(float q0[3], float q1[3]);

public:
    void interpolate(float q0[3], ReedsSheppPath &path, float seg, float q[3]);
    
};
#endif