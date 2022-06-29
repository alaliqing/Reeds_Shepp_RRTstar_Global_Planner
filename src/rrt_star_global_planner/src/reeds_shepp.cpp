#include "reeds_shepp.hpp"
#include <iostream>
#include <boost/math/constants/constants.hpp>


namespace
{
    const float pi = boost::math::constants::pi<float>();
    const float RS_EPS = 1e-3;
    const float ZERO = 10*std::numeric_limits<float>::epsilon();
    
    inline float mod2pi(float theta)
    {
        float v = fmod(theta, 2. * pi);
        if (v < -pi)
            v += 2. * pi;
        else
            if(v > pi)
                v -= 2 * pi;
        return v;
    }

    inline void polar(float x, float y, float &r, float &theta)
    {
        r = sqrt(x * x + y * y);
        theta = atan2(y, x);
    }

    inline void tauOmega(float u, float v, float xi, float eta, float phi, float &tau, float &omega)
    {
        float delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
        float t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
        tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
        omega = mod2pi(tau - u + v - phi);
    }

    inline bool LSL(float x, float y, float phi, float &t, float &u, float &v)
    {
        polar(x - sin(phi), y - 1. + cos(phi), u, t);
        if (t >= -ZERO)
        {
            v = mod2pi(phi - t);
            if (v >= -ZERO)
            {
                assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
                assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
                return true;
            }
        }
    return false;   
    } 

    inline bool LSR(float x, float y, float phi, float &t, float &u, float &v)
    {
        float t1, u1;
        polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
        u1 = u1 * u1;
        if (u1 >= 4.)
        {
            float theta;
            u = sqrt(u1 - 4.);
            theta = atan2(2., u);
            t = mod2pi(t1 + theta);
            v = mod2pi(t - phi);
            assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
            assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
            return t >= -ZERO && v >= -ZERO;
        }
        return false;
    }


    void CSC(float x, float y, float phi, ReedsShepp::ReedsSheppPath &path)
    {
        float t, u, v, Lmin = path.length(), L;
        if (LSL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[14], t, u, v);
            Lmin = L;
        }
        if (LSL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[14], -t, -u, -v);
            Lmin = L;
        }
        if (LSL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[15], t, u, v);
            Lmin = L;
        }
        if (LSL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[15], -t, -u, -v);
            Lmin = L;
        }
        if (LSR(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[12], t, u, v);
            Lmin = L;
        }
        if (LSR(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[12], -t, -u, -v);
            Lmin = L;
        }
        if (LSR(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[13], t, u, v);
            Lmin = L;
        }
        if (LSR(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[13], -t, -u, -v);
    }

    inline bool LRL(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
        polar(xi, eta, u1, theta);
        if (u1 <= 4.)
        {
            u = -2. * asin(.25 * u1);
            t = mod2pi(theta + .5 * u + pi);
            v = mod2pi(phi - t + u);
            assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
            assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO;
        }
        return false;
    }

    void CCC(float x, float y, float phi, ReedsShepp::ReedsSheppPath &path)
    {
        float t, u, v, Lmin = path.length(), L;
        if (LRL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[0], t, u, v);
            Lmin = L;
        }
        if (LRL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[0], -t, -u, -v);
            Lmin = L;
        }
        if (LRL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[1], t, u, v);
            Lmin = L;
        }
        if (LRL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[1], -t, -u, -v);
            Lmin = L;
        }

        // backwards
        float xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LRL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[0], v, u, t);
            Lmin = L;
        }
        if (LRL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[0], -v, -u, -t);
            Lmin = L;
        }
        if (LRL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[1], v, u, t);
            Lmin = L;
        }
        if (LRL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[1], -v, -u, -t);
    }

    inline bool LRLRn(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
        if (rho <= 1.)
        {
            u = acos(rho);
            tauOmega(u, -u, xi, eta, phi, t, v);
            assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
            assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
            return t >= -ZERO && v <= ZERO;
        }
        return false;
    }

    inline bool LRLRp(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
        if (rho >= 0 && rho <= 1)
        {
            u = -acos(rho);
            if (u >= -.5 * pi)
            {
                tauOmega(u, u, xi, eta, phi, t, v);
                assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
                assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                return t >= -ZERO && v >= -ZERO;
            }
        }
        return false;
    }

    void CCCC(float x, float y, float phi, ReedsShepp::ReedsSheppPath &path)
    {
        float t, u, v, Lmin = path.length(), L;
        if (LRLRn(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[2], t, u, -u, v);
            Lmin = L;
        }
        if (LRLRn(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[2], -t, -u, u, -v);
            Lmin = L;
        }
        if (LRLRn(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[3], t, u, -u, v);
            Lmin = L;
        }
        if (LRLRn(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[3], -t, -u, u, -v);
            Lmin = L;
        }

        if (LRLRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[2], t, u, u, v);
            Lmin = L;
        }
        if (LRLRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[2], -t, -u, -u, -v);
            Lmin = L;
        }
        if (LRLRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[3], t, u, u, v);
            Lmin = L;
        }
        if (LRLRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[3], -t, -u, -u, -v);
    }

    inline bool LRSL(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            float r = sqrt(rho * rho - 4.);
            u = 2. - r;
            t = mod2pi(theta + atan2(r, -2.));
            v = mod2pi(phi - .5 * pi - t);
            assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
            assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO && v <= ZERO;
        }
        return false;
    }

    inline bool LRSR(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(-eta, xi, rho, theta);
        if (rho >= 2.)
        {
            t = theta;
            u = 2. - rho;
            v = mod2pi(t + .5 * pi - phi);
            assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
            assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
            assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
            return t >= -ZERO && u <= ZERO && v <= ZERO;
        }
        return false;
    }

    void CCSC(float x, float y, float phi, ReedsShepp::ReedsSheppPath &path)
    {
        float t, u, v, Lmin = path.length() - .5 * pi, L;
        if (LRSL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[4], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LRSL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[4], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LRSL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[5], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LRSL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[5], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        if (LRSR(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[8], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LRSR(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[8], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LRSR(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[9], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LRSR(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[9], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        // backwards
        float xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LRSL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[6], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LRSL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[6], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LRSL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[7], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LRSL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[7], -v, -u, .5 * pi, -t);
            Lmin = L;
        }

        if (LRSR(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[10], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LRSR(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[10], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LRSR(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[11], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LRSR(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[11], -v, -u, .5 * pi, -t);
    }

    inline bool LRSLR(float x, float y, float phi, float &t, float &u, float &v)
    {
        float xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
        polar(xi, eta, rho, theta);
        if (rho >= 2.)
        {
            u = 4. - sqrt(rho * rho - 4.);
            if (u <= ZERO)
            {
                t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                v = mod2pi(t - phi);
                assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
                assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                return t >= -ZERO && v >= -ZERO;
            }
        }
        return false;
    }

    void CCSCC(float x, float y, float phi, ReedsShepp::ReedsSheppPath &path)
    {
        float t, u, v, Lmin = path.length() - pi, L;
        if (LRSLR(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[16], t, -.5 * pi, u, -.5 * pi, v);
            Lmin = L;
        }
        if (LRSLR(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[16], -t, .5 * pi, -u, .5 * pi, -v);
            Lmin = L;
        }
        if (LRSLR(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[17], t, -.5 * pi, u, -.5 * pi, v);
            Lmin = L;
        }
        if (LRSLR(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
            path = ReedsShepp::ReedsSheppPath(
                ReedsShepp::PathType[17], -t, .5 * pi, -u, .5 * pi, -v);
    }
}

ReedsShepp::ReedsSheppPath reedsShepp(float x, float y, float phi)
{
    ReedsShepp::ReedsSheppPath path;
    CSC(x, y, phi, path);
    CCC(x, y, phi, path);
    CCCC(x, y, phi, path);
    CCSC(x, y, phi, path);
    CCSCC(x, y, phi, path);

    return path;
}

const ReedsShepp::SegmentType
ReedsShepp::PathType[18][5] =
{
    { LEFT, RIGHT, LEFT, NOP, NOP },             // 0
    { RIGHT, LEFT, RIGHT, NOP, NOP },            // 1
    { LEFT, RIGHT, LEFT, RIGHT, NOP },           // 2
    { RIGHT, LEFT, RIGHT, LEFT, NOP },           // 3
    { LEFT, RIGHT, STRAIGHT, LEFT, NOP },        // 4
    { RIGHT, LEFT, STRAIGHT, RIGHT, NOP },       // 5
    { LEFT, STRAIGHT, RIGHT, LEFT, NOP },        // 6
    { RIGHT, STRAIGHT, LEFT, RIGHT, NOP },       // 7
    { LEFT, RIGHT, STRAIGHT, RIGHT, NOP },       // 8
    { RIGHT, LEFT, STRAIGHT, LEFT, NOP },        // 9
    { RIGHT, STRAIGHT, RIGHT, LEFT, NOP },       // 10
    { LEFT, STRAIGHT, LEFT, RIGHT, NOP },        // 11
    { LEFT, STRAIGHT, RIGHT, NOP, NOP },         // 12
    { RIGHT, STRAIGHT, LEFT, NOP, NOP },         // 13
    { LEFT, STRAIGHT, LEFT, NOP, NOP },          // 14
    { RIGHT, STRAIGHT, RIGHT, NOP, NOP },        // 15
    { LEFT, RIGHT, STRAIGHT, LEFT, RIGHT },      // 16
    { RIGHT, LEFT, STRAIGHT, RIGHT, LEFT }       // 17
};

ReedsShepp::ReedsSheppPath::ReedsSheppPath(const SegmentType* type,
    float t, float u, float v, float w, float x)
    : type_(type)
{
    length_[0] = t;
    length_[1] = u;
    length_[2] = v;
    length_[3] = w;
    length_[4] = x;
    totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}


float ReedsShepp::distance(float q0[3], float q1[3])
{
    return rho_ * reedsShepp(q0, q1).length();
}

ReedsShepp::ReedsSheppPath ReedsShepp::reedsShepp(float q0[3], float q1[3])
{
    float dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
    float c = cos(q0[2]), s = sin(q0[2]);
    float x = c * dx + s * dy, y = -s * dx + c * dy;

    return ::reedsShepp(x / rho_, y / rho_, dth);
}

std::vector<int>ReedsShepp::ReedsPathtype(float q0[3], float q1[3])
{
    ReedsSheppPath path = reedsShepp(q0, q1);
    std::vector<int>types;
    for (int i=0;i<5;++i)
    {
        types.push_back(int(path.type_[i]));
        std::cout<<path.type_[i]<<std::endl;
    }
    return  types ;
}

void ReedsShepp::interpolate(float q0[3], ReedsSheppPath &path, float seg, float s[3])
{

    if (seg < 0.0) seg = 0.0;
    if (seg > path.length()) seg = path.length();

    float phi, v;

    s[0] = s[1] = 0.0;
    s[2] = q0[2];

    for (unsigned int i = 0; i < 5 && seg > 0; ++i)
    {
        if (path.length_[i] < 0)
        {
            v = std::max(-seg, path.length_[i]);
            seg += v;
        }
        else
        {
            v = std::min(seg, path.length_[i]);
            seg -= v;
        }
        phi = s[2];
        switch(path.type_[i])
        {
            case LEFT:
                s[0] += (sin(phi + v) - sin(phi));
                s[1] += (-cos(phi + v) + cos(phi));
                s[2] = phi + v;
                break;
            case RIGHT:
                s[0] += (-sin(phi-v) + sin(phi));
                s[1] += ( cos(phi-v) - cos(phi));
                s[2] = phi - v;
                break;
            case STRAIGHT:
                s[0] += (v * cos(phi));
                s[1] += (v * sin(phi));
                break;
            case NOP:
                break;  
        }
    }

    s[0] = s[0] * rho_ + q0[0];
    s[1] = s[1] * rho_ + q0[1];
}

std::vector<std::vector<float>> ReedsShepp::ReedsPathsample(float q0[], float q1[], float step_size, float& result_length,
                                                            std::vector<float>& result_x, std::vector<float>& result_y, std::vector<float>& result_yaw)
{
    ReedsSheppPath path = reedsShepp(q0, q1);
    float dist = rho_ * path.length();
    result_length = dist;
    std::vector<std::vector<float> > result;
    for (float seg=0.0; seg<=dist; seg+=step_size)
    {
        float qnew[3] = {};
        interpolate(q0, path, seg/rho_, qnew);
        std::vector<float> temp;
        for(int i=0;i<3;i++)
        {
            temp.push_back(qnew[i]);
        }
//        std::cout<<qnew[0]<<"   "<<qnew[1]<<"  "<<qnew[2]<<std::endl;
        result_x.push_back(qnew[0]);
        result_y.push_back(qnew[1]);
        result_yaw.push_back(qnew[2]);
        result.push_back(temp);
    }
    return result;
}

// int main()
// {
//     ReedsShepp RS;
//     float q0[3] = {15.63, 27.95, 1.57};
//     float q1[3] = {19.152431, 28.528984, 0.473168};
//     float step = 0.1;
//     float result_length;
// //    std::cout<<result_length<<std::endl;

//     std::vector<float> result_x;
//     std::vector<float> result_y;
//     std::vector<std::vector<float>> result;
//     result = RS.ReedsPathsample(q0, q1, step, result_length, result_x, result_y);
// //    std::cout<<result[(result.size()-1)][0]<<", "<<result[(result.size()-1)][1]<<", "<<result[(result.size()-1)][2]<<std::endl;
// //    std::cout<<result.back()[0]<<", "<<result.back()[1]<<", "<<result.back()[2]<<std::endl;
//     return 0;
// }
