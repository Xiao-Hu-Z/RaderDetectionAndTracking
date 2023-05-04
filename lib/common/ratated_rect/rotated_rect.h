#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "auto_buffer.h"

constexpr double kEpsilon = 1e-12;

template <typename _Tp> class Point_ {
  public:
    typedef _Tp value_type;

    // various constructors
    Point_();
    Point_(_Tp _x, _Tp _y);
    Point_(const Point_ &pt);
    _Tp x, y;
};

typedef Point_<float> Point2f;

class RotatedRect {
  public:
    RotatedRect() = default;
    ~RotatedRect() = default;

  private:
    float CrossProduct(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c);
    float DotProduct(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c);
    float Distance(Eigen::Vector2d a, Eigen::Vector2d b);

    int top = 0;
    double min_sqr = 1e12;

    void Graham(std::vector<Eigen::Vector2d> &points,
                std::vector<Eigen::Vector2d> &stack);

    void RotatingCalipers(std::vector<Eigen::Vector2d> &points,
                          std::vector<Eigen::Vector2d> &ans);
    void NewRotatingCalipers(std::vector<Point2f> &points, int n, float *out);

  public:
    Eigen::Vector2d center;
    Eigen::Vector2d size;
    float angle;

    void ComputeOrientation(std::vector<Eigen::Vector2d> &points);
    void NewComputeOrientation(std::vector<Eigen::Vector2d> &points);
};
