#pragma once 

#include <iostream>
#include <map>
#include <mutex>
#include <string>

#include <yaml-cpp/yaml.h>

#include "../configure/rader_demo_config.h"
#include "../log.h"


class ConfigManager {
  public:
    ~ConfigManager();

  private:
    // 私有化默认构造函数、禁止拷贝构造和拷贝赋值
    ConfigManager();
    ConfigManager(const ConfigManager &) = delete;
    ConfigManager &operator=(const ConfigManager &) = delete;

    // thread-safe interface.
    bool Init();

    // thread-safe interface.
    bool Reset();
    size_t NumModels() const { return model_config_map_.size(); }

  private:
    bool InitInternal();

    // key: model_name
    std::map<std::string, std::string> model_config_map_;
    std::mutex mutex_; // multi-thread init safe.
    bool inited_ = false;

  public:
    // 局部静态变量单例模式
    bool GetModelConfig(const std::string &model_name,
                        std::string &model_config);
    static ConfigManager *GetInstance();

    bool ReadYaml(const std::string &file_name, YAML::Node &config);
};
