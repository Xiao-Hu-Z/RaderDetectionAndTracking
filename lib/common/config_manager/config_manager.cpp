#include "config_manager.h"


ConfigManager::ConfigManager() {}
ConfigManager::~ConfigManager() {}

bool ConfigManager::Init() {
    std::lock_guard<std::mutex> lck(mutex_);
    return InitInternal();
}

bool ConfigManager::InitInternal() {
    if (inited_) {
        return true;
    }
    model_config_map_.clear();

    std::string yaml_module_path = CONFIG_FILE_PATH;
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_module_path);
    } catch (YAML::Exception &ex) {
        printf("yaml_module_path : %s get file list error.\n", yaml_module_path.c_str());
    }
    // LPPrintInfo << "config_file_path: " << yaml_module_path;
    printf("config_file_path: %s\n", yaml_module_path.c_str());

    // 将各模块功能类名和配置参数文件路径存储在字典 model_config_map_ 中
    std::string root_path = ROOT_PATH;
    for (YAML::Node::iterator it = config.begin(); it != config.end(); it++) {
        model_config_map_[it->first.as<std::string>()] =
            root_path + "/" + it->second.as<std::string>();
    }

    // LPPrintInfo << "finish to load ModelConfigs. NumModels: "
    //             << model_config_map_.size();
    LPPrintInfo("finish to load ModelConfigs. NumModels: %ld\n", model_config_map_.size());
    inited_ = true;

    return true;
}

bool ConfigManager::GetModelConfig(const std::string &model_name,
                                   std::string &model_config) {
    if (!inited_ && !Init()) {
        return false;
    }

    auto citer = model_config_map_.find(model_name);
    if (citer == model_config_map_.end()) {
        return false;
    }
    model_config = citer->second;
    return true;
}

ConfigManager *ConfigManager::GetInstance() {
    static ConfigManager obj;
    return &obj;
}

bool ConfigManager::ReadYaml(const std::string &file_name, YAML::Node &config) {
    try {
        config = YAML::LoadFile(file_name);
    } catch (YAML::Exception &ex) {
        return false;
    }
    return true;
}

