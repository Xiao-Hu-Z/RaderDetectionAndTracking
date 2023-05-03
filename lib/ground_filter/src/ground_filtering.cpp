/*
* @Author: Hua_zhi
* @Date: 2022-08-15
* @Brief description: region ground segmentation, reference Patchwork
* @@Last modified Hua_zhi
*/
#include "ground_filter/ground_filtering.h"

//作用域申明，采用三层作用域
namespace leap{
namespace lidar{


GroundFilter::GroundFilter(){}

GroundFilter::~GroundFilter(){}

bool GroundFilter::Init(){

    std::string config_file;
    auto config_maneger = ConfigManager::GetInstance();
    // LPPrintCHECK(config_maneger->GetModelConfig(Name(), config_file));
    config_maneger->GetModelConfig(Name(), config_file);   

    bool ret = Init(config_file); 

    return true;
}


bool GroundFilter::Init(const std::string config_file){

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (YAML::Exception &ex)
    {
        // LPPrintErr << "The yaml file path is not correct!" << config_file ;  
        LPPrintErr("The ground filter file %s is not correct!", config_file.c_str());  
        return false;
    }
    LPPrintInfo("ground filter read yaml success\n");

    sensor_height_ = config["sensor_height"].as<float>();
    verbose_ = config["verbose"].as<bool>();

    num_iter_ = config["num_iter"].as<int>();
    num_lpr_ = config["num_lpr"].as<int>();
    num_min_pts_ = config["num_min_pts"].as<int>();

    th_seeds_ = config["th_seeds"].as<float>();
    th_dist_ = config["th_dist"].as<float>();
    th_seeds_v_ = config["th_seeds_v"].as<float>();
    th_dist_v_ = config["th_dist_v"].as<float>();

    max_range_ = config["max_range"].as<float>();
    min_range_ = config["min_range"].as<float>();

    uprightness_thr_ = config["uprightness_thr"].as<float>();
    adaptive_seed_selection_margin_ = config["adaptive_seed_selection_margin"].as<float>();

    using_global_thr_ = config["using_global_thr"].as<bool>();
    global_elevation_thr_ = config["global_elevation_thr"].as<float>();

    num_zones_ = config["num_zones"].as<int>();
    num_rings_each_zone_ = config["num_rings_each_zone"].as<std::vector<int>>();
    num_sectors_each_zone_ = config["num_sectors_each_zone"].as<std::vector<int>>();
    min_ranges_ = config["min_ranges"].as<std::vector<float>>();

    elevation_thr_ = config["elevation_thr"].as<std::vector<float>>();
    flatness_thr_ = config["flatness_thr"].as<std::vector<float>>();

    // It equals to elevation_thr_.size()/flatness_thr_.size();
    num_rings_of_interest_ = elevation_thr_.size();

    revert_pc_.reserve(3000);
    ground_pc_.reserve(3000);
    non_ground_pc_.reserve(3000);
    regionwise_ground_.reserve(3000);
    regionwise_nonground_.reserve(3000);

    min_range_z2_ = min_ranges_[1];
    min_range_z3_ = min_ranges_[2];
    min_range_z4_ = min_ranges_[3];

    min_ranges_   = {min_range_, min_range_z2_, min_range_z3_, min_range_z4_};
    ring_sizes_   = {(min_range_z2_ - min_range_) / num_rings_each_zone_.at(0),
                        (min_range_z3_ - min_range_z2_) / num_rings_each_zone_.at(1),
                        (min_range_z4_ - min_range_z3_) / num_rings_each_zone_.at(2),
                        (max_range_ - min_range_z4_) / num_rings_each_zone_.at(3)};
    sector_sizes_ = {2 * M_PI / num_sectors_each_zone_.at(0), 2 * M_PI / num_sectors_each_zone_.at(1),
                        2 * M_PI / num_sectors_each_zone_.at(2),
                        2 * M_PI / num_sectors_each_zone_.at(3)};

    for (int iter = 0; iter < num_zones_; ++iter) {
        Zone z;
        InitializeZone(z, num_sectors_each_zone_.at(iter), num_rings_each_zone_.at(iter));
        ConcentricZoneModel_.push_back(z);
    }
    // LPPrintInfo << "INITIALIZATION COMPLETE";
    LPPrintInfo("INITIALIZATION COMPLETE");

    return true;
}

//提取初始化种子点
void GroundFilter::ExtractInitialSeeds(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds, float th_seed)
{
    init_seeds.points.clear();

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    int init_idx = 0;
    //自适应种子点选择，Z1效果好，Z2~Z4高度都会上漂
    static double lowest_h_margin_in_close_zone = (sensor_height_ == 0.0)? -0.1 : adaptive_seed_selection_margin_ * sensor_height_;
    if (zone_idx == 0) {
        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < lowest_h_margin_in_close_zone) {
                ++init_idx;
            } else {
                break;
            }
        }
    }

    // 计算点云平均高度，选择20个点
    for (int i          = init_idx; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0

    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        //找寻种子点
        if (p_sorted.points[i].z < lpr_height + th_seed) {
            init_seeds.points.push_back(p_sorted.points[i]);
        }
    }
}
//估计地面参数
void GroundFilter::EstimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground, float th_dist) {
    //计算点云标准化协方差
    pcl::computeMeanAndCovarianceMatrix(ground, cov_, pc_mean_);//点云协方差+质心
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);
    singular_values_ = svd.singularValues();

    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean_.head<3>();//质心坐标向量

    // according to normal.T*[x,y,z] = -d
    //Matrix_convert_float//warning
    //坐标原点到质心与法向量构成平面的距离
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // std::cout << "d_ is "<< d_ << std::endl;
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist - d_;
}
//初始化Zone
void GroundFilter::InitializeZone(Zone &z, int num_sectors, int num_rings)
{
    z.clear();
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(1000);//预留1000个点
    Ring     ring;
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(cloud);
    }
    for (int j = 0; j < num_rings; j++) {
        z.emplace_back(ring);
    }

}

inline void GroundFilter::FlushPatchesInZone(Zone &patches, int num_sectors, int num_rings)
{
  for (int i = 0; i < num_sectors; i++)
  {
    for (size_t j = 0; j < num_rings; j++)
    {
      if(!patches[j][i].points.empty()) patches[j][i].points.clear();//清空
    }
    
  }
}

inline float GroundFilter::CalculateTheta(const float x, const float y)
{ // 0 ~ 2 * PI
    if (y >= 0) {
        return atan2(y, x); // 1, 2 quadrant
    } else {
        return 2 * M_PI + atan2(y, x);// 3, 4 quadrant
    }
}
//点云映射，分块
void GroundFilter::CaculateCzm(const pcl::PointCloud<pcl::PointXYZI> &src, std::vector<Zone> &czm) {

    for (auto const &pt : src.points) {
        int    ring_idx, sector_idx;
        double r = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
        if ((r <= max_range_) && (r > min_range_)) {
            float theta = CalculateTheta(pt.x, pt.y);//计算方位角

            if (r < min_range_z2_) { // In First rings
                ring_idx   = std::min(static_cast<int>(((r - min_range_) / ring_sizes_[0])), num_rings_each_zone_[0] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[0])), num_sectors_each_zone_[0] - 1);
                czm[0][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z3_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z2_) / ring_sizes_[1])), num_rings_each_zone_[1] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[1])), num_sectors_each_zone_[1] - 1);
                czm[1][ring_idx][sector_idx].points.emplace_back(pt);
            } else if (r < min_range_z4_) {
                ring_idx   = std::min(static_cast<int>(((r - min_range_z3_) / ring_sizes_[2])), num_rings_each_zone_[2] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[2])), num_sectors_each_zone_[2] - 1);
                czm[2][ring_idx][sector_idx].points.emplace_back(pt);
            } else { // Far!
                ring_idx   = std::min(static_cast<int>(((r - min_range_z4_) / ring_sizes_[3])), num_rings_each_zone_[3] - 1);
                sector_idx = std::min(static_cast<int>((theta / sector_sizes_[3])), num_sectors_each_zone_[3] - 1);
                czm[3][ring_idx][sector_idx].points.emplace_back(pt);
            }
        }

    }
}
//提取块域地面
void GroundFilter::ExtractPieceWiseGround(
        const int zone_idx, const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &non_ground_dst) 
{
    // Timer check_time("check_time");
    // 0. Initialization
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!dst.empty()) dst.clear();
    if (!non_ground_dst.empty()) non_ground_dst.clear();
    
    //R-VPF
    //构建非地面点索引并初始化空间
    // std::vector<bool> nonground_idx;
    // nonground_idx.resize(src.points.size(),false);
    // for(int i=0; i < num_iter_; ++i){
    //     //创建垂直点云空间,并赋值
    //     pcl::PointCloud<pcl::PointXYZI> src_wo_verticals;
    //     for(int j=0; j < nonground_idx.size(); ++j){
    //         if (!nonground_idx[j]) src_wo_verticals.points.push_back(src.points[j]);
    //     }
    //     ExtractInitialSeeds(zone_idx, src_wo_verticals, ground_pc_, th_seeds_v_);
    //     EstimatePlane(ground_pc_, th_dist_v_);//估计平面参数
    //     //pointcloud to matrix
    //     Eigen::MatrixXf points_v(src.points.size(), 3);
    //     int             j      = 0;
    //     for (auto &p:src.points) {
    //         points_v.row(j++) << p.x, p.y, p.z;
    //     }
    //     // ground plane model
    //     //地面模型，所有点的地面高度
    //     Eigen::VectorXf result_v = points_v * normal_;
    //     //低于垂直度阈值60°，可能为垂直面
    //     if (zone_idx == 0 && normal_(2, 0) < uprightness_thr_)
    //     {
    //         // std::cout << "run the R-VPF!" <<std::endl;
    //         for ( int r=0; r<nonground_idx.size(); r++ ) {
    //             if (!nonground_idx[r]) {
    //                 if ( abs(result_v[r]) < th_dist_v_) {
    //                     nonground_idx[r] == true;//过滤
    //                     non_ground_dst.push_back(src[r]);
    //                 }
    //             }   
    //         }
    //     }
    //     else break;
    // }
    // R-GPF
    // pcl::PointCloud<pcl::PointXYZI> src_wo_nongrounds;
    // for ( int r=0; r<nonground_idx.size(); r++ )
    // {
    //     if (!nonground_idx[r]) src_wo_nongrounds.push_back(src[r]);
    // }
    // 1. set seeds!
    // Timer InitialSeeds("InitialSeeds");
    // ExtractInitialSeeds(zone_idx, src_wo_nongrounds, ground_pc_, th_seeds_);
    ExtractInitialSeeds(zone_idx, src, ground_pc_, th_seeds_);
    // InitialSeeds.TicToc();
    // 2. Extract ground
    //迭代3次地面估计
    for (int i = 0; i < num_iter_; i++) {
        //std::cout<<"the size of seed ground:"<<ground_pc_.points.size()<<std::endl;
        EstimatePlane(ground_pc_, th_dist_);//估计平面参数
        ground_pc_.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j      = 0;
        for (auto       &p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        //地面模型，所有点的地面高度
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        //迭代次数之前不断更新地面点的数据
        for (int r = 0; r < result.rows(); r++) {
            if (i < num_iter_ - 1) {
                if (result[r] < th_dist_d_) {
                    ground_pc_.points.push_back(src[r]);
                }
            } else { // Final stage
                if (result[r] < th_dist_d_) {
                    dst.points.push_back(src[r]);
                } else {
                    if (i == num_iter_ - 1) {
                        non_ground_dst.push_back(src[r]);
                    }
                }
            }
        }
    }
    // check_time.TicToc();
}

//estimate ground
void GroundFilter::EstimateGround(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud)
        // double &time_taken) 
{
    // Timer forward_time("forward_time");
    // LPPrintInfo << "filtered ground start!";
    // 1.Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    laserCloudIn = *cloud_in;

    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(), laserCloudIn.end(), [](pcl::PointXYZI a, pcl::PointXYZI b){
        return a.z < b.z;
    });

    // 3.Error point removal
    // As there are some error mirror reflection under the ground,
    // here regardless point under 1.8* sensor_height
    // Sort point according to height, here uses z-axis in default
    auto     it = laserCloudIn.points.begin();
    for (int i  = 0; i < laserCloudIn.points.size(); i++) {
        if (laserCloudIn.points[i].z < -1.8 * sensor_height_) {
            it++;
        } else {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);//删除低于1.8*sensor_height

    // t1 = ros::Time::now().toSec();
    // 4. pointcloud -> regionwise setting
    for (int k = 0; k < num_zones_; ++k) {//置零，三维空间
        // Timer flushpatch("flushpatch");
        FlushPatchesInZone(ConcentricZoneModel_[k], num_sectors_each_zone_[k], num_rings_each_zone_[k]);
        // flushpatch.TicToc();
    }
    // Timer calculate_czm_time_("calculate_czm_time");
    CaculateCzm(laserCloudIn, ConcentricZoneModel_);//点云映射,降维
    // calculate_czm_time_.TicToc();
    pcl::PointCloud<pcl::PointXYZI> cloud_nonground;
    pcl::PointCloud<pcl::PointXYZI> cloud_out;

    cloud_out.clear();
    cloud_nonground.clear();
    revert_pc_.clear();
    revert_pc_.clear();
    
    // forward_time.TicToc();
    int      concentric_idx = 0;
    for (int k              = 0; k < num_zones_; ++k) {
        // Timer run_zone_time("run_zone_time");
        auto          zone     = ConcentricZoneModel_[k];
        for (uint16_t ring_idx = 0; ring_idx < num_rings_each_zone_[k]; ++ring_idx) {
            for (uint16_t sector_idx = 0; sector_idx < num_sectors_each_zone_[k]; ++sector_idx) {
                if (zone[ring_idx][sector_idx].points.size() > num_min_pts_) {//区域点数量限制
                    // Timer ExtractPiece("ExtractPiece");
                    ExtractPieceWiseGround(k, zone[ring_idx][sector_idx], regionwise_ground_, regionwise_nonground_);
                    // ExtractPiece.TicToc();
                    // Status of each patch
                    // used in checking uprightness, elevation, and flatness, respectively
                    const double ground_z_vec       = abs(normal_(2, 0));//法向量的Z值
                    const double ground_z_elevation = pc_mean_(2, 0);//质心的Z值
                    const double surface_variable   =
                                         singular_values_.minCoeff() /
                                         (singular_values_(0) + singular_values_(1) + singular_values_(2));
                    if (ground_z_vec < uprightness_thr_) {
                        // All points are rejected
                        cloud_nonground += regionwise_ground_;
                        cloud_nonground += regionwise_nonground_;
                    } else { // satisfy uprightness 
                        if (concentric_idx < num_rings_of_interest_) {//仅仅选取XX个同心索引
                            if (ground_z_elevation > elevation_thr_[concentric_idx]) {//高程阈值
                                // 增加高度阈值，可以消除近处
                                if ((flatness_thr_[concentric_idx] > surface_variable) && (ground_z_elevation < global_elevation_thr_)) {//平面度阈值
                                    if (verbose_) {
                                        // LPPrintInfo << "Recovery operated. Check "
                                        //           << concentric_idx
                                        //           << "th param. flatness_thr_: " << flatness_thr_[concentric_idx]
                                        //           << " > "
                                        //           << surface_variable;
                                        revert_pc_ += regionwise_ground_;
                                    }
                                    cloud_out += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                } else {
                                    if (verbose_) {
                                        // LPPrintInfo <<"Rejection operated. Check"
                                        //           << concentric_idx
                                        //           << "th param. of elevation_thr_: " << elevation_thr_[concentric_idx]
                                        //           << " < "
                                        //           << ground_z_elevation;
                                        reject_pc_ += regionwise_ground_;
                                    }
                                    cloud_nonground += regionwise_ground_;
                                    cloud_nonground += regionwise_nonground_;
                                }
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        } else {
                            if (using_global_thr_ && (ground_z_elevation > global_elevation_thr_)) {//高程阈值
                                // LPPrintInfo << "[Global elevation] " << ground_z_elevation << " > " << global_elevation_thr_;
                                cloud_nonground += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            } else {
                                cloud_out += regionwise_ground_;
                                cloud_nonground += regionwise_nonground_;
                            }
                        }
                        
                    }

                }
            }
            ++concentric_idx;
        }
        // run_zone_time.TicToc();
    }
    *ground_cloud = cloud_out;
    *non_ground_cloud = cloud_nonground;
}

}//lidar
}//leap



