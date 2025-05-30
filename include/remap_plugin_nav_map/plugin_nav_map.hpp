// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REMAP_PLUGIN_NAV_MAP__PLUGIN_NAV_MAP_HPP_
#define REMAP_PLUGIN_NAV_MAP__PLUGIN_NAV_MAP_HPP_

#include <tf2/LinearMath/Vector3.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>

#include <pal_stores_client/stores_client.hpp>
#include <nlohmann/json.hpp>

namespace remap
{
namespace plugins
{
class PluginNavMap : public SemanticPlugin
{
private:
  double walls_height_ {2.5};
  bool map_stored_ {false};
  bool map_processed_ {false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> zone_map_;

  std::vector<std::pair<tf2::Vector3, tf2::Vector3>> computePolygonBoundingBox(
    const std::vector<tf2::Vector3> & polygon);
  std::vector<std::pair<tf2::Vector3, tf2::Vector3>> computePolygonBoundingBox(
    const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon);
  tf2::Vector3 computeRayCastingSource(
    const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & bb);
  int countIntersections(
    const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon,
    const tf2::Vector3 & candidate,
    const tf2::Vector3 & ray_cast_source);

  void fillPolygon(
    const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon,
    const std::string & region);

  // debugging
  void printPolygon(const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon);
  void printVector(const tf2::Vector3 & polygon);

public:
  PluginNavMap();
  PluginNavMap(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register);
  ~PluginNavMap();
  void run() override;
  void initialize() override;
};
}   // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_NAV_MAP__PLUGIN_NAV_MAP_HPP_
