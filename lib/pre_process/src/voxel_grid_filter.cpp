
#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter() {}

bool VoxelGridFilter::Init() {

    auto config_manager = ConfigManager::GetInstance();
    std::string config_file;
    // LPPrintCHECK(config_manager->GetModelConfig(Name(), config_file));
    config_manager->GetModelConfig(Name(), config_file);

    // YAML::Node config;
    // try {
    //     config = YAML::LoadFile(config_file);
    // } catch (YAML::Exception &ex) {
    //     // LPPrintErr << "The yaml file path is not correct! " << config_file;
    //     LPPrintErr("The yaml file %s is not correct! ", config_file.c_str());
    //     return false;
    // }
    // is_downsample_ = config["is_downsample"].as<bool>();
    // leaf_size_x_ = config["leaf_size_x"].as<float>();
    // leaf_size_y_ = config["leaf_size_y"].as<float>();
    // leaf_size_z_ = config["leaf_size_z"].as<float>();

    bool ret = Init(config_file);

    return ret;
}

bool VoxelGridFilter::Init(const std::string config_file) {
    
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);        
    }
    catch (YAML::Exception &ex)
    {
        PrintErr << "The yaml file path is not correct!" << config_file ;  
        return false;
    }

    PrintInfo << "cluster read yaml success";


    is_downsample_ = config["is_downsample"].as<bool>();
    leaf_size_x_ = config["leaf_size_x"].as<float>();
    leaf_size_y_ = config["leaf_size_y"].as<float>();
    leaf_size_z_ = config["leaf_size_z"].as<float>();

    return true;
}

void VoxelGridFilter::DownSample(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud) {
    if (is_downsample_ && leaf_size_x_ >= 0.1 && leaf_size_y_ >= 0.1 &&
        leaf_size_z_ >= 0.1) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(in_cloud);
        voxel.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
        voxel.filter(*out_cloud);
    } else {
        out_cloud = in_cloud;
    }
}