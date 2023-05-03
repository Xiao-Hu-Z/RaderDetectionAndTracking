#pragma once

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include "../../common/config_manager/config_manager.h"
#include "../../common/log.h"


class RoiClip {
  public:
    RoiClip();
    ~RoiClip(){};

    bool Init();
    bool Init(const std::string config_file);
    bool GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &out);

  private:
    // params
    int filter_naninf_points_ = true;
    bool filter_nearby_box_points_ = true;
    float roi_x_min_;
    float roi_x_max_;
    float roi_y_min_;
    float roi_y_max_;
    float roi_z_max_;
    float vehicle_x_min_;
    float vehicle_x_max_;
    float vehicle_y_min_;
    float vehicle_y_max_;

    bool IsIn(const float x, const float x_min, const float x_max) {
        return (x < x_max) && (x > x_min);
    }

        std::string Name() const { return "RoiClip"; }
};
