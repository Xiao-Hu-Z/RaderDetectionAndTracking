#include "common/convex_hull.h"

constexpr double kMathEpsilon = 1e-12;

float ConvexHull::CrossProd(const Eigen::Vector2d &p1,
                            const Eigen::Vector2d &p2,
                            const Eigen::Vector2d &p3) {
    Eigen::Vector2d p12 = p2 - p1;
    Eigen::Vector2d p13 = p3 - p1;
    return p12(0) * p13(1) - p12(1) * p13(0);
}

bool ConvexHull::ComputeConvexHull(std::vector<Eigen::Vector2d> &in,
                                   std::vector<Eigen::Vector2d> &hull) {
    if (in.empty())
        return false;
    const int n = static_cast<int>(in.size());
    if (n < 3) {
        return false;
    }

    std::vector<int> sorted_indices(n);
    for (int i = 0; i < n; ++i) {
        sorted_indices[i] = i;
    }
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](const int idx1, const int idx2) {
                  auto pt1 = in[idx1];
                  auto pt2 = in[idx2];
                  const float dx = pt1[0] - pt2[0];
                  if (std::abs(dx) > kMathEpsilon) {
                      return dx < 0.0;
                  }
                  return pt1[1] < pt2[1];
              });
    int count = 0;
    std::vector<int> results;
    results.reserve(n);
    int last_count = 1;
    for (int i = 0; i < n + n; ++i) {
        if (i == n) {
            last_count = count;
        }
        const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
        Eigen::Vector2d pt = in[idx];
        while (count > last_count &&
               CrossProd(in[results[count - 2]], in[results[count - 1]], pt) <=
                   kMathEpsilon) {
            results.pop_back();
            --count;
        }
        results.push_back(idx);
        ++count;
    }
    --count;
    if (count < 3) {
        return false;
    }

    for (int i = 0; i < count; ++i) {
        hull.push_back(in[results[i]]);
    }
    return true;
}
