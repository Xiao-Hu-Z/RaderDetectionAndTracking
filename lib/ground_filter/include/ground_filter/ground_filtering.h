/*
* @Author: Hua_zhi
* @Date: 2022-08-15
* @Brief description: region ground segmentation, reference Patchwork
* @Last modified Hua_zhi
*/
#ifndef GROUND_FILTERING_H_
#define GROUND_FILTERING_H_

#include <iostream>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

#include <Eigen/Eigen>

#include "common/config_manager.h"
// #include "common/log.h
#include "common/lpslog.h"

//作用域定义，采用三层作用域
namespace leap{
namespace lidar{
//类型定义
typedef std::vector<pcl::PointCloud<pcl::PointXYZI>> Ring;
typedef std::vector<Ring> Zone;
//定义类体
class GroundFilter
{
    public:
        GroundFilter();        
        ~GroundFilter();
        bool Init(); 
        bool Init(const std::string config_file);                          
        void EstimateGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud);
    private:
        //CZM module
        int num_iter_;
        int num_lpr_;
        int num_min_pts_;
        int num_zones_;
        int num_rings_of_interest_;

        float sensor_height_;
        //R-GPF
        float th_seeds_;
        float th_dist_;
        //R-VPF
        float th_seeds_v_;
        float th_dist_v_;

        float max_range_;
        float min_range_;
        float uprightness_thr_;
        float adaptive_seed_selection_margin_;
        float min_range_z2_;
        float min_range_z3_;
        float min_range_z4_;
        bool verbose_;
        
        // For global threshold
        bool   using_global_thr_;
        float global_elevation_thr_;
        //RPF module
        float           d_;
        Eigen::MatrixXf normal_;
        Eigen::VectorXf singular_values_;
        float           th_dist_d_;
        Eigen::Matrix3f cov_;
        Eigen::Vector4f pc_mean_;

        std::vector<int> num_sectors_each_zone_;
        std::vector<int> num_rings_each_zone_;

        std::vector<float> sector_sizes_;
        std::vector<float> ring_sizes_;
        std::vector<float> min_ranges_;
        std::vector<float> elevation_thr_;
        std::vector<float> flatness_thr_;

        std::vector<Zone> ConcentricZoneModel_;

        pcl::PointCloud<pcl::PointXYZI> revert_pc_, reject_pc_;
        pcl::PointCloud<pcl::PointXYZI> ground_pc_;
        pcl::PointCloud<pcl::PointXYZI> non_ground_pc_;

        pcl::PointCloud<pcl::PointXYZI> regionwise_ground_;
        pcl::PointCloud<pcl::PointXYZI> regionwise_nonground_;

    private:
        void ExtractInitialSeeds(
            const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
            pcl::PointCloud<pcl::PointXYZI> &init_seeds, float th_seed);
        void EstimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground, float th_dist);
        float CalculateTheta(const float x, const float y);
        void InitializeZone(Zone &z, int num_sectors, int num_rings);
        void FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings);
        void CaculateCzm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm);
        void ExtractPieceWiseGround(const int zone_idx,
                                const pcl::PointCloud<pcl::PointXYZI> &src,
                                pcl::PointCloud<pcl::PointXYZI> &dst,
                                pcl::PointCloud<pcl::PointXYZI> &non_groud_dst);
        std::string Name() const{ return "GroundFilter";}

};


}//lidar
}//leap

#endif