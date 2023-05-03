#include "object_builder.h"


#define EPS 1e-10

void ObjectBuilder::Build(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                          std::vector<RadarObstacle> &cluster_objects) {
    if (in == nullptr || in->empty()) {
        return;
    }
    Eigen::Vector3f max_pt, min_pt;
    std::vector<Eigen::Vector2d> cluster_points;
    GetMinMax3D(in, max_pt, min_pt, cluster_points);
    RadarObstacle object;
    GetPositionAndSize(object, max_pt, min_pt);
    object.confidence = 1;
    std::vector<Eigen::Vector2d> hulls;
    convex_hull_.ComputeConvexHull(cluster_points, hulls);
    object.polygon_vertex_num = 2 * hulls.size();
    for (auto i = 0; i < 2 * hulls.size() - 1; i += 2) {
        object.polygon_vertex[i].x =
            hulls[i / 2 % object.polygon_vertex_num][0];
        object.polygon_vertex[i].y =
            hulls[i / 2 % object.polygon_vertex_num][1];
        object.polygon_vertex[i].z = min_pt[2];
        object.polygon_vertex[i + 1].x =
            hulls[i / 2 % object.polygon_vertex_num][0];
        object.polygon_vertex[i + 1].y =
            hulls[i / 2 % object.polygon_vertex_num][1];
        object.polygon_vertex[i + 1].z = max_pt[2];
    }

    // 待修改
    min_area_rect_.NewComputeOrientation(hulls);
}

void ObjectBuilder::Build(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                          RadarObstacle &object) {
    if (in == nullptr || in->empty()) {
        return;
    }
    Eigen::Vector3f max_pt, min_pt;
    std::vector<Eigen::Vector2d> cluster_points;
    GetMinMax3D(in, max_pt, min_pt, cluster_points);
    GetPositionAndSize(object, max_pt, min_pt);

    object.confidence = 1;

    std::vector<Eigen::Vector2d> hulls;
    convex_hull_.ComputeConvexHull(cluster_points, hulls);
    object.polygon_vertex_num = 2 * hulls.size();
    for (auto i = 0; i < 2 * hulls.size() - 1; i += 2) {
        object.polygon_vertex[i].x =
            hulls[i / 2 % object.polygon_vertex_num][0];
        object.polygon_vertex[i].y =
            hulls[i / 2 % object.polygon_vertex_num][1];
        object.polygon_vertex[i].z = min_pt[2];
        object.polygon_vertex[i + 1].x =
            hulls[i / 2 % object.polygon_vertex_num][0];
        object.polygon_vertex[i + 1].y =
            hulls[i / 2 % object.polygon_vertex_num][1];
        object.polygon_vertex[i + 1].z = max_pt[2];
    }
}

void ObjectBuilder::GetMinMax3D(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Eigen::Vector3f &max_pt,
    Eigen::Vector3f &min_pt, std::vector<Eigen::Vector2d> &in_points) {
    min_pt[0] = min_pt[1] = min_pt[2] = std::numeric_limits<float>::max();
    max_pt[0] = max_pt[1] = max_pt[2] = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (std::isnan(cloud->points[i].x) || std::isnan(cloud->points[i].y) ||
            std::isnan(cloud->points[i].z)) {
            continue;
        }

        min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
        min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
        min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
        max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
        max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
        max_pt[2] = std::max(max_pt[2], cloud->points[i].z);

        in_points.emplace_back(
            Eigen::Vector2d{cloud->points[i].x, cloud->points[i].y});
    }
}

void ObjectBuilder::GetPositionAndSize(RadarObstacle &object,
                                       Eigen::Vector3f &max_pt,
                                       Eigen::Vector3f &min_pt) {
    object.position.x = (min_pt[0] + max_pt[0]) / 2;
    object.position.y = (min_pt[1] + max_pt[1]) / 2;
    object.position.z = (min_pt[2] + max_pt[2]) / 2;

    float length_ = max_pt[0] - min_pt[0];
    float width_ = max_pt[1] - min_pt[1];
    float height_ = max_pt[2] - min_pt[2];

    object.size.length = ((length_ < 0) ? -1 * length_ : length_);
    object.size.width = ((width_ < 0) ? -1 * width_ : width_);
    object.size.height = ((height_ < 0) ? -1 * height_ : height_);
}
