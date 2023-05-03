#include "roi_clip.h"

RoiClip::RoiClip() {}

bool RoiClip::Init() {
    std::string config_file;
    auto config_manager = ConfigManager::GetInstance();
    config_manager->GetModelConfig(Name(), config_file);
    bool ret = Init(config_file);

    return ret;
}


bool RoiClip::Init(const std::string config_file) {
   
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

    
    filter_naninf_points_ = config["filter_naninf_points"].as<bool>();
    filter_nearby_box_points_ = config["filter_nearby_box_points"].as<bool>();
    roi_x_min_ = config["roi_x_min"].as<float>();
    roi_x_max_ = config["roi_x_max"].as<float>();
    roi_y_min_ = config["roi_y_min"].as<float>();
    roi_y_max_ = config["roi_y_max"].as<float>();
    roi_z_max_ = config["roi_z_max"].as<float>();

    vehicle_x_min_ = config["vehicle_x_min"].as<float>();
    vehicle_x_max_ = config["vehicle_x_max"].as<float>();
    vehicle_y_min_ = config["vehicle_y_min"].as<float>();
    vehicle_y_max_ = config["vehicle_y_max"].as<float>();

    return true;
}

bool RoiClip::GetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &out) {
    if (in == nullptr || in->points.empty()) {
        return false;
    }

    for (auto &pt : in->points) {
        if (filter_naninf_points_) {
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                continue;
            }
        }
        // if ((IsIn(pt.x, roi_x_min_, roi_x_max_) &&
        //      IsIn(pt.y, roi_y_min_, roi_y_max_) && pt.z < roi_z_max_) &&
        //     !(IsIn(pt.x, vehicle_x_min_, vehicle_x_max_) &&
        //      IsIn(pt.y, vehicle_y_min_, vehicle_y_max_))) {
        //     out->push_back(pt);
        // }
        if (!IsIn(pt.x, roi_x_min_, roi_x_max_) ||
            !IsIn(pt.y, roi_y_min_, roi_y_max_) || pt.z > roi_z_max_) {
            continue;
        }

        if (filter_nearby_box_points_ && pt.x < vehicle_x_max_ &&
            pt.x > vehicle_x_min_ && pt.y < vehicle_y_max_ &&
            pt.y > vehicle_y_min_) {
            continue;
        }
        out->push_back(pt);
    }

    return true;
}

