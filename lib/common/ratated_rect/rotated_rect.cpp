/*
 * @Author: zhao_xiaohu
 * @Date: 2022-09-06 09:12:17
 * @Last Modified by:   zhao_xiaohu
 * @Last Modified time: 2022-09-08 09:12:17
 */
#include "common/rotated_rect.h"
#include "common/out_lidar_data_type.h"


template <typename _Tp> inline Point_<_Tp>::Point_() : x(0), y(0) {}

template <typename _Tp>
inline Point_<_Tp>::Point_(_Tp _x, _Tp _y) : x(_x), y(_y) {}

template <typename _Tp>
inline Point_<_Tp>::Point_(const Point_ &pt) : x(pt.x), y(pt.y) {}



// int Dcmp(double x) {
//     if (fabs(x) < kEpsilon)
//         return 0;
//     if (x > -kEpsilon)
//         return 1;
//     return -1;
// }

// float RotatedRect::CrossProduct(Eigen::Vector2d a, Eigen::Vector2d b,
//                                 Eigen::Vector2d c) {
//     return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
// }

// float RotatedRect::DotProduct(Eigen::Vector2d a, Eigen::Vector2d b,
//                               Eigen::Vector2d c) {
//     return (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]);
// }

// float RotatedRect::Distance(Eigen::Vector2d a, Eigen::Vector2d b) {
//     return sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]));
// }

// void RotatedRect::Graham(std::vector<Eigen::Vector2d> &points,
//                          std::vector<Eigen::Vector2d> &stack) {
//     sort(points.begin(), points.end(),
//          [&](Eigen::Vector2d a, Eigen::Vector2d b) {
//              if (!Dcmp(a[0] - b[0]))
//                  return a[1] < b[1];
//              return a[0] < b[0];
//          });
//     for (int i = 0; i < points.size(); i++) //下凸壳
//     {
//         while (top >= 2 &&
//                Dcmp(CrossProduct(stack[top - 1], stack[top], points[i]) <= 0))
//             top--;
//         stack[++top] = points[i];
//     }
//     int tmp = top;
//     for (int i = points.size() - 1; i >= 0; i--) //上凸壳
//     {
//         while (top >= tmp + 1 &&
//                Dcmp(CrossProduct(stack[top - 1], stack[top], points[i]) <= 0))
//             top--;
//         stack[++top] = points[i];
//     }
//     top--; // 栈顶显然是points[1],重复了一次，将重复的丢掉
// }

// void RotatedRect::RotatingCalipers(
//     std::vector<Eigen::Vector2d> &stack,
//     std::vector<Eigen::Vector2d> &ans) // 旋转卡壳
// {
//     int left = 1, right = 1, up = 1; // 左、右、上三个点
//     for (int i = 1; i <= top; i++) {
//         while (Dcmp(CrossProduct(stack[up + 1], stack[i], stack[i + 1]) -
//                     CrossProduct(stack[up], stack[i], stack[i + 1])) >= 0)
//             up = up % top + 1;
//         while (Dcmp(DotProduct(stack[right + 1], stack[i + 1], stack[i]) -
//                     DotProduct(stack[right], stack[i + 1], stack[i])) >= 0)
//             right = right % top + 1;
//         if (i == 1)
//             left = right;
//         while (Dcmp(DotProduct(stack[left + 1], stack[i], stack[i + 1]) -
//                     DotProduct(stack[left], stack[i], stack[i + 1])) >= 0)
//             left = left % top + 1;
//         double L = Distance(stack[i], stack[i + 1]); // L=点i到i+1的距离
//         double H =
//             CrossProduct(stack[i + 1], stack[up], stack[i]) / L; // H是矩形的高
//         double bottom =
//             (fabs(DotProduct(stack[i], stack[i + 1], stack[left]) / L) +
//              fabs(DotProduct(stack[i], stack[i + 1], stack[right]) / L));
//         double nowsqr = bottom * H;
//         if (nowsqr < min_sqr) //这个解比当前答案更优
//         {
//             min_sqr = nowsqr;
//             ans[0] =
//                 stack[i] +
//                 (stack[i + 1] - stack[i]) *
//                     ((fabs(DotProduct(stack[i], stack[i + 1], stack[right])) /
//                           L +
//                       L) /
//                      L);
//             ans[1] = ans[0] + (stack[right] - ans[0]) *
//                                   (H / Distance(stack[right], ans[0]));
//             ans[2] = ans[1] + (stack[up] - ans[1]) *
//                                   (bottom / Distance(stack[up], ans[1]));
//             ans[3] = ans[2] + (stack[left] - ans[2]) *
//                                   (H / Distance(stack[left], ans[2]));
//         }
//     }
// }

// bool operator<(Eigen::Vector2d a, Eigen::Vector2d b) {
//     if (!Dcmp(a[1] - b[1]))
//         return a[0] < b[0];
//     return a[1] < b[1];
// }

// void RotatedRect::ComputeOrientation(std::vector<Eigen::Vector2d> &points) {
//     int len = points.size();
//     std::vector<Eigen::Vector2d> stack(len + 2);
//     std::vector<Eigen::Vector2d> ans(4);
//     Graham(points, stack);
//     // 顶点逆时针
//     RotatingCalipers(stack, ans);

//     sort(ans.begin(), ans.end(),
//          [&](Eigen::Vector2d a, Eigen::Vector2d b) { return a[0] > b[0]; });

//     // for (int i = 0; i <= 3; i++) {
//     //     printf("%.5lf %.5lf\n", ans[i][0], ans[i][1]);
//     // }

//     center[0] = (ans[0][0] + ans[1][0] + ans[2][0] + ans[3][0]) / 4;
//     center[1] = (ans[0][1] + ans[1][1] + ans[2][1] + ans[3][1]) / 4;

//     if (ans[0][1] < ans[1][1]) {
//         angle = (float)atan2(ans[3][1] - ans[3][1], ans[3][0] - ans[3][0]);
//         size[0] = Distance(ans[0], ans[1]);
//         size[1] = Distance(ans[3], ans[0]);
//     } else {
//         angle = (float)atan2(ans[1][1] - ans[0][1], ans[1][0] - ans[0][0]);
//         size[1] = Distance(ans[0], ans[1]);
//         size[0] = Distance(ans[3], ans[0]);
//     }
// }

void RotatedRect::NewRotatingCalipers(std::vector<Point2f> &points, int n,
                                      float *out) {
    float minarea = 1e12;
    float max_dist = 0;
    char buffer[32] = {};
    int i, k;
    AutoBuffer<float> abuf(n * 3);
    float *inv_vect_length = abuf.data(); // 生成指针
    Point2f *vect = (Point2f *)(inv_vect_length + n);
    int left = 0, bottom = 0, right = 0, top = 0;
    int seq[4] = {-1, -1, -1, -1};

    /* rotating calipers sides will always have coordinates
     (a,b) (-b,a) (-a,-b) (b, -a)
     */
    /* this is a first base vector (a,b) initialized by (1,0) */
    float orientation = 0;
    float base_a;
    float base_b = 0;

    float left_x, right_x, top_y, bottom_y;
    Point2f pt0 = points[0];

    left_x = right_x = pt0.x;
    top_y = bottom_y = pt0.y;

    for (i = 0; i < n; i++) {
        double dx, dy;

        if (pt0.x < left_x)
            left_x = pt0.x, left = i;

        if (pt0.x > right_x)
            right_x = pt0.x, right = i;

        if (pt0.y > top_y)
            top_y = pt0.y, top = i;

        if (pt0.y < bottom_y)
            bottom_y = pt0.y, bottom = i;

        Point2f pt = points[(i + 1) & (i + 1 < n ? -1 : 0)];

        dx = pt.x - pt0.x;
        dy = pt.y - pt0.y;

        vect[i].x = (float)dx;
        vect[i].y = (float)dy;
        inv_vect_length[i] = (float)(1. / std::sqrt(dx * dx + dy * dy));

        pt0 = pt;
    }

    // find convex hull orientation
    double ax = vect[n - 1].x;
    double ay = vect[n - 1].y;

    for (i = 0; i < n; i++) {
        double bx = vect[i].x;
        double by = vect[i].y;

        double convexity = ax * by - ay * bx;

        if (convexity != 0) {
            orientation = (convexity > 0) ? 1.f : (-1.f);
            break;
        }
        ax = bx;
        ay = by;
    }
    if (fabs(orientation) < 1e-10) {
        return;
    }

    base_a = orientation;

    /*  init calipers position */
    seq[0] = bottom;
    seq[1] = right;
    seq[2] = top;
    seq[3] = left;

    /*   Main loop - evaluate angles and rotate calipers   */

    /* all of edges will be checked while rotating calipers by 90 degrees */
    for (k = 0; k < n; k++) {
        /* sinus of minimal angle */
        /*float sinus;*/

        /* compute cosine of angle between calipers side and polygon edge */
        /* dp - dot product */
        float dp[4] = {
            +base_a * vect[seq[0]].x + base_b * vect[seq[0]].y,
            -base_b * vect[seq[1]].x + base_a * vect[seq[1]].y,
            -base_a * vect[seq[2]].x - base_b * vect[seq[2]].y,
            +base_b * vect[seq[3]].x - base_a * vect[seq[3]].y,
        };

        float maxcos = dp[0] * inv_vect_length[seq[0]];

        /* number of calipers edges, that has minimal angle with edge */
        int main_element = 0;

        /* choose minimal angle */
        for (i = 1; i < 4; ++i) {
            float cosalpha = dp[i] * inv_vect_length[seq[i]];
            if (cosalpha > maxcos) {
                main_element = i;
                maxcos = cosalpha;
            }
        }

        /*rotate calipers*/
        {
            // get next base
            int pindex = seq[main_element];
            float lead_x = vect[pindex].x * inv_vect_length[pindex];
            float lead_y = vect[pindex].y * inv_vect_length[pindex];
            switch (main_element) {
            case 0:
                base_a = lead_x;
                base_b = lead_y;
                break;
            case 1:
                base_a = lead_y;
                base_b = -lead_x;
                break;
            case 2:
                base_a = -lead_x;
                base_b = -lead_y;
                break;
            case 3:
                base_a = -lead_y;
                base_b = lead_x;
                break;
            default:
                printf("main_element should be 0, 1, 2 or 3");
            }
        }
        seq[main_element] += 1;
        seq[main_element] = (seq[main_element] == n) ? 0 : seq[main_element];
        {
            float height;
            float area;

            float dx = points[seq[1]].x - points[seq[3]].x;
            float dy = points[seq[1]].y - points[seq[3]].y;
            float width = dx * base_a + dy * base_b;
            dx = points[seq[2]].x - points[seq[0]].x;
            dy = points[seq[2]].y - points[seq[0]].y;

            height = -dx * base_b + dy * base_a;

            area = width * height;
            if (area <= minarea) {
                float *buf = (float *)buffer;

                minarea = area;
                ((int *)buf)[0] = seq[3];
                buf[1] = base_a;
                buf[2] = width;
                buf[3] = base_b;
                buf[4] = height;
                ((int *)buf)[5] = seq[0];
                buf[6] = area;
            }
        }
        break;
    }

    {
        float *buf = (float *)buffer;

        float A1 = buf[1];
        float B1 = buf[3];

        float A2 = -buf[3];
        float B2 = buf[1];

        float C1 =
            A1 * points[((int *)buf)[0]].x + points[((int *)buf)[0]].y * B1;
        float C2 =
            A2 * points[((int *)buf)[5]].x + points[((int *)buf)[5]].y * B2;

        float idet = 1.f / (A1 * B2 - A2 * B1);

        float px = (C1 * B2 - C2 * B1) * idet;
        float py = (A1 * C2 - A2 * C1) * idet;

        out[0] = px;
        out[1] = py;

        out[2] = A1 * buf[2];
        out[3] = B1 * buf[2];

        out[4] = A2 * buf[4];
        out[5] = B2 * buf[4];
    }
}

void RotatedRect::NewComputeOrientation(std::vector<Eigen::Vector2d> &p) {
    Point2f out[3];
    int n = p.size();
    std::vector<Point2f> points(n);
    for (int i = 0; i < n; i++) {
        points[i].x = p[i][0];
        points[i].y = p[i][1];
    }

    if (n > 2) {
        NewRotatingCalipers(points, n, (float *)out);
        center[0] = out[0].x + (out[1].x + out[2].x) * 0.5f;
        center[1] = out[0].y + (out[1].y + out[2].y) * 0.5f;
        size[1] = (float)std::sqrt((double)out[1].x * out[1].x +
                                   (double)out[1].y * out[1].y);
        size[0] = (float)std::sqrt((double)out[2].x * out[2].x +
                                   (double)out[2].y * out[2].y);
        angle = (float)atan2((double)out[1].y, (double)out[1].x);
    } else if (n == 2) {
        center[0] = (points[0].x + points[1].x) * 0.5f;
        center[1] = (points[0].y + points[1].y) * 0.5f;
        double dx = points[1].x - points[0].x;
        double dy = points[1].y - points[0].y;
        size[1] = (float)std::sqrt(dx * dx + dy * dy);
        size[0] = 0;
        angle = (float)atan2(dy, dx);
    } else {
        if (n == 1) {
            center[1] = points[0].x;
            center[0] = points[0].y;
        }
    }
}
