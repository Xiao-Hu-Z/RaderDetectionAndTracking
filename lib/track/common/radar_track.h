/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <lib/common/object.h>

namespace apollo {
namespace perception {
namespace radar {

class RadarTrack {
 public:
  RadarTrack(const ObjectPtr &obs, const double timestamp);
  ~RadarTrack() {}
  // update the object after association with a radar obervation
  void UpdataObsRadar(const ObjectPtr &obs_radar, const double timestamp);
  void SetObsRadarNullptr();
  int GetObsId() const;
  ObjectPtr GetObsRadar();
  ObjectPtr GetObs();
  double GetTimestamp();
  double GetTrackingTime();
  bool IsDead() { return is_dead_; }
  void SetDead() { is_dead_ = true; }
  bool ConfirmTrack() { return tracked_times_ > s_tracked_times_threshold_; }
  static void SetTrackedTimesThreshold(const int &threshold) {
    s_tracked_times_threshold_ = threshold;
  }
  static void SetChosenFilter(const std::string &chosen_filter) {
    s_chosen_filter_ = chosen_filter;
  }
  static void SetUseFilter(bool use_filter) { s_use_filter_ = use_filter; }

 private:
  double timestamp_ = 0.0;
  int obs_id_ = 0;
  int tracked_times_ = 0;
  double tracking_time_ = 0.0;
  bool is_dead_ = false;
  ObjectPtr obs_radar_ = nullptr;  // observasion from radar
  ObjectPtr obs_ = nullptr;        // track result after tracking
  std::shared_ptr<BaseFilter> filter_ = nullptr;

  static std::string s_chosen_filter_;
  static int s_current_idx_;
  static int s_tracked_times_threshold_;
  static bool s_use_filter_;

  // 声明私有的拷贝构造函数和赋值构造函数，但不去定义实现它们
  // 1.声明了拷贝构造函数和赋this数，阻止了编译器暗自创建的专属版本．
  // 2.声明了private,阻止了外this它们的调用．
  // 3.不定义它们，可以保证成员函数和友元函数调用它们时，产生一个连接错误．
  // DISALLOW_COPY_AND_ASSIGN(RadarTrack);
};

typedef std::shared_ptr<RadarTrack> RadarTrackPtr;

}  // namespace radar
}  // namespace perception
}  // namespace apollo
