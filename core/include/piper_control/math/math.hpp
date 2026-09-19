#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

#include "piper_control/common/constants.hpp"

namespace piper_control
{

template <typename T>
inline T clamp(T value, T low, T high)
{
    if (low > high)
    {
        std::swap(low, high);
    }

    return std::clamp(value, low, high);
}

template <typename Derived>
inline Derived trim_small(
    const Eigen::MatrixBase<Derived>& matrix,
    double eps = 1e-10)
{
    Derived result = matrix.derived();

    for (Eigen::Index i = 0; i < result.rows(); ++i)
    {
        for (Eigen::Index j = 0; j < result.cols(); ++j)
        {
            if (std::abs(result(i, j)) < eps)
            {
                result(i, j) = 0.0;
            }
        }
    }

    return result;
}

inline double deg_to_rad(double deg)
{
    return deg * kPi / 180.0;
}

inline double rad_to_deg(double rad)
{
    return rad * 180.0 / kPi;
}

}  // namespace piper_control