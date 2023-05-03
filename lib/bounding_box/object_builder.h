
#pragma once

#include <algorithm>
#include <future>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/io.h>

#include "common/common.h"
#include "common/convex_hull.h"
#include "common/out_lidar_data_type.h"
#include "common/rotated_rect.h"


class ObjectBuilder {
  public:
    ObjectBuilder() = default;
    ~ObjectBuilder() = default;
    void Build(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
               std::vector<RadarObstacle> &cluster_objects);
    void Build(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
               RadarObstacle &object);
    void GetMinMax3D(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                     Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt,
                     std::vector<Eigen::Vector2d> &in_points);
    void GetPositionAndSize(RadarObstacle &object, Eigen::Vector3f &max_pt,
                            Eigen::Vector3f &min_pt);

  private:
  private:
    std::mutex mutex_;
    ConvexHull convex_hull_;
    RotatedRect min_area_rect_;
};
