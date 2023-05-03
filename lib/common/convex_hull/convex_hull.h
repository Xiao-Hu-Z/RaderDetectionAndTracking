/*
 * @Author: zhao_xiaohu
 * @Date: 2022-08-20 03:06:25
 * @Last Modified by:   zhao_xiaohu
 * @Last Modified time: 2022-08-20 03:06:25
 */
#pragma once

#include <algorithm>
#include <math.h>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/common.h"
#include "common/out_lidar_data_type.h"

namespace leap {
namespace lidar {

constexpr double MathEpsilon = 1e-9;

class ConvexHull {
  public:
    ConvexHull() = default;
    ~ConvexHull() = default;

    float CrossProd(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2,
                    const Eigen::Vector2d &p3);
    bool ComputeConvexHull(std::vector<Eigen::Vector2d> &in,
                           std::vector<Eigen::Vector2d> &hull);
    bool OldComputeConvexHull(std::vector<Eigen::Vector2d> &in_points,
                              std::vector<Eigen::Vector2d> &hull);
};
} // namespace lidar
} // namespace leap
