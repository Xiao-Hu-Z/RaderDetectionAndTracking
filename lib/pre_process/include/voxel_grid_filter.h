#pragma once

#include <string>

#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>

#include "../../common/config_manager/config_manager.h"
#include "../../common/log.h"

class VoxelGridFilter {
  public:
    VoxelGridFilter();
    ~VoxelGridFilter(){};

    bool Init();
    bool Init(const std::string config_file);
    void DownSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud);

  private:
    //下采样：体素滤波参数
    float leaf_size_x_;
    float leaf_size_y_;
    float leaf_size_z_;
    bool is_downsample_;

    std::string Name() const { return "VoxelGridFilter"; }
};

