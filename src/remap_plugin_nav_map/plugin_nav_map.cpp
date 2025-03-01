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

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>

#include <cmath>

#include "remap_plugin_nav_map/plugin_nav_map.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

namespace remap
{
namespace plugins
{
PluginNavMap::PluginNavMap()
: SemanticPlugin() {}

PluginNavMap::PluginNavMap(
  std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
  std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
: SemanticPlugin(semantic_map, regions_register),
  walls_height_(2.5),
  map_stored_(false),
  map_processed_(false) {}

PluginNavMap::~PluginNavMap()
{
  timer_.reset();
  semantic_map_.reset();
  regions_register_.reset();
}

void PluginNavMap::initialize()
{
  rclcpp::QoS map_qos(1);
  map_qos.reliable();
  map_qos.transient_local();


  // stores_client_ = std::make_shared<stores::StoresClient>(node_ptr_.get());

  /*
  map_sub_ = node_ptr_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    map_qos,
    std::bind(&PluginNavMap::mapCallback, this, std::placeholders::_1));
  */

  try {
    // Get package share directory
    std::string package_name = "remap_plugin_nav_map";
    std::string package_path = ament_index_cpp::get_package_prefix(package_name);

    // Path to YAML file
    std::string yaml_file = "/home/user/ex/src/remap_plugin_nav_map/maps/small_house.yaml";

    // Load YAML file
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Data structure to store zone vertices

    // Parse YAML zones
    if (config["zones"]) {
      for (const auto& zone : config["zones"]) {
        std::string zone_name = zone["name"].as<std::string>();
        std::vector<geometry_msgs::msg::PoseStamped> vertices;

        if (zone["vertexes"]) {
          for (const auto& vertex : zone["vertexes"]) {
            geometry_msgs::msg::PoseStamped pose_stamped;

            // Fill pose timestamp (set current time, here simulated as 0)
            pose_stamped.header.stamp = rclcpp::Time(0);
            pose_stamped.header.frame_id = "map"; // Frame ID

            // Extract translation
            pose_stamped.pose.position.x = vertex["transform"]["translation"]["x"].as<double>();
            pose_stamped.pose.position.y = vertex["transform"]["translation"]["y"].as<double>();
            pose_stamped.pose.position.z = vertex["transform"]["translation"]["z"].as<double>();

            // Extract rotation (Quaternion)
            pose_stamped.pose.orientation.x = vertex["transform"]["rotation"]["x"].as<double>();
            pose_stamped.pose.orientation.y = vertex["transform"]["rotation"]["y"].as<double>();
            pose_stamped.pose.orientation.z = vertex["transform"]["rotation"]["z"].as<double>();
            pose_stamped.pose.orientation.w = vertex["transform"]["rotation"]["w"].as<double>();

            vertices.push_back(pose_stamped);
          }
        }

        // Store in map
        zone_map_[zone_name] = vertices;
        this->pushFact(zone_name + " rdf:type ZOI");
      }
    } else {
        std::cerr << "No 'zones' found in the YAML file." << std::endl;
    }
    for (const auto& [zone_name, poses] : zone_map_) {
      std::cout << "Zone: " << zone_name << " (Vertices: " << poses.size() << ")" << std::endl;
      for (const auto& pose : poses) {
        std::cout << "  - Position: (" << pose.pose.position.x << ", "
                  << pose.pose.position.y << ", " << pose.pose.position.z << ")"
                  << std::endl;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "Error loading YAML file: " << e.what() << std::endl;
      return;
    }

    map_stored_ = true;
}

void PluginNavMap::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  // Testing: we just read if a map is received
  // and we build the walls; we do not expect any update
  // on the map
  RCLCPP_INFO(node_ptr_->get_logger(), "Map stored");
  map_ = *map;
}

void PluginNavMap::run()
{
  // Here we check if we have already inserted the map into the grid;
  // If we have to do it, then we iterate over the map
  // and we add for each cell in the grid a voxel in the map
  // using the SemanticMapHandler activateVoxel function.
  // We do this remembering that the representation in remap
  // is centre-centered by defaul, that is, for the 
  // element 0, 0 of the map, we want to activate the voxel
  // corresponding to the centre of that cell in the VDB grid

  // At this stage, being this the first version of the plugin,
  // we do not accept any reference frame for the map
  // if not the fixed frame of the MapHandler

  // auto kitchen = stores_client_->loadRaw("/small_house/small_house_floor/kitchen", "eulero::Zone");

  if ((!map_stored_) || (map_stored_ && map_processed_)) {
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Running");

  for (const auto & room : zone_map_) {
    std::string room_name = room.first;
    std::vector<std::pair<tf2::Vector3, tf2::Vector3>> vertex_couples;
    for (size_t i = 0; i < room.second.size(); ++i) {
      size_t i_p = (i + 1) % room.second.size();
      vertex_couples.push_back({
        {room.second[i].pose.position.x,
         room.second[i].pose.position.y,
         room.second[i].pose.position.z},
        {room.second[i_p].pose.position.x,
         room.second[i_p].pose.position.y,
         room.second[i_p].pose.position.z}});
    }
    for (const auto & vertex_couple : vertex_couples) {
      auto & wall_start = vertex_couple.first;
      auto & wall_end = vertex_couple.second;
      tf2::Vector3 wall_direction = wall_end - wall_start;
      auto wall_length = wall_direction.length();
      auto wall_direction_normalized = wall_direction;
      wall_direction_normalized.normalize();
      for (float length = 0.0; length < wall_length; length += 0.05) {
        tf2::Vector3 wall_point = wall_start + wall_direction_normalized * length;
        for (double height = 0.0; height < walls_height_; height += 0.05) {
          wall_point.setZ(height);
          semantic_map_->insertVoxel(
          wall_point.x(),
          wall_point.y(),
          wall_point.z(),
          room_name,
          *regions_register_);
        }
      }
    }
    fillPolygon(vertex_couples, room_name);
  }
  
  RCLCPP_INFO(node_ptr_->get_logger(), "Map processed");
  map_processed_ = true;
}

std::vector<std::pair<tf2::Vector3, tf2::Vector3>> PluginNavMap::computePolygonBoundingBox(
    const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon)
{
  std::vector<tf2::Vector3> unwrapped_polygon;
  for (const auto & segment : polygon) {
    unwrapped_polygon.push_back(segment.first);
  }

  return computePolygonBoundingBox(unwrapped_polygon);
}

std::vector<std::pair<tf2::Vector3, tf2::Vector3>> PluginNavMap::computePolygonBoundingBox(
    const std::vector<tf2::Vector3> & polygon)
{
  std::vector<std::pair<tf2::Vector3, tf2::Vector3>> bb;
  if (polygon.size() == 0) {
    return bb;
  }

  auto x_min = polygon[0].x();
  auto y_min = polygon[0].y();
  auto x_max = polygon[0].x();
  auto y_max = polygon[0].y();

  for (const auto & corner : polygon) {
    x_min = (corner.x() < x_min) ? corner.x() : x_min;
    y_min = (corner.y() < y_min) ? corner.y() : y_min;
    x_max = (corner.x() > x_max) ? corner.x() : x_max;
    y_max = (corner.y() > y_max) ? corner.y() : y_max;
  }

  bb.push_back({{x_min, y_min, 0.0}, {x_max, y_min, 0.0}});
  bb.push_back({{x_max, y_min, 0.0}, {x_max, y_max, 0.0}});
  bb.push_back({{x_max, y_max, 0.0}, {x_min, y_max, 0.0}});
  bb.push_back({{x_min, y_max, 0.0}, {x_min, y_min, 0.0}});

  return bb;
}

tf2::Vector3 PluginNavMap::computeRayCastingSource(
  const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & bb)
{
  // This function expects to receive a bounding box
  // where the first element is the pair relative to the
  // lowest segment
  tf2::Vector3 ray_cast_source;
  if (bb.size() > 0) {
    ray_cast_source.setX((bb[0].first.x()+bb[0].second.x())/2.0);
    ray_cast_source.setY(bb[0].first.y()-1.0);
    ray_cast_source.setZ(0.0);
  }
  return ray_cast_source;
}

void PluginNavMap::printPolygon(const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon)
{
  for (const auto & segment : polygon) {
    printVector(segment.first);
    std::cout<<" ";
  }
  std::cout<<"\n";
}

void PluginNavMap::printVector(const tf2::Vector3 & point)
{
  std::cout<<"{"<<point.x()<<", "<<point.y()<<", "<<point.z()<<"}";
}

int PluginNavMap::countIntersections(
  const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon,
  const tf2::Vector3 & candidate,
  const tf2::Vector3 & ray_cast_source)
{
  int intersections = 0;
  double confidence_delta = 1e-3;

  // first, we compute the equation for the ray casting line
  auto m_rc = (candidate.y() - ray_cast_source.y()) / (candidate.x() - ray_cast_source.x());
  auto q_rc = candidate.y() - candidate.x() * m_rc;

  // then, we evaluate the intersection with every segment of the polygon
  for (const auto & segment : polygon) {
    // we compute the line equation for each segment
    auto m_s = (segment.first.y() - segment.second.y()) / (segment.first.x() - segment.second.x());
    auto q_s = segment.first.y() - segment.first.x() * m_s;

    // we compute the intersection between the two lines
    if (std::abs(m_rc - m_s) - confidence_delta < 0.0) {
      continue;
    }
    auto x_int = (q_s - q_rc) / (m_rc - m_s);
    auto y_int = x_int * m_rc + q_rc;
    tf2::Vector3 intersection_p = {x_int, y_int, 0.0};

    // we compute whether the intersection is part of the segment or not
    auto segment_direction = segment.second - segment.first;
    auto intersection_direction = intersection_p - segment.first;
    int sd_x_sign = static_cast<int>(
      std::round(segment_direction.x() / std::abs(segment_direction.x())));
    int id_x_sign = static_cast<int>(
      std::round(intersection_direction.x() / std::abs(intersection_direction.x())));
    if (sd_x_sign != id_x_sign) {
      continue;
    }
    if ((intersection_direction.length() <= segment_direction.length()) 
      && ((candidate-ray_cast_source).length() > (intersection_p-ray_cast_source).length())) {
      intersections++;
    }
  }

  return intersections;
}

void PluginNavMap::fillPolygon(
  const std::vector<std::pair<tf2::Vector3, tf2::Vector3>> & polygon,
  const std::string & region)
{
  double step_size = 0.1;

  if (polygon.size() < 3) {
    return;
  }
  auto bb = computePolygonBoundingBox(polygon);
  auto ray_cast_source = computeRayCastingSource(bb);
  for (double p_y = bb[0].first.y(); p_y <= bb[3].first.y(); p_y += step_size) {
    for (double p_x = bb[0].first.x(); p_x <= bb[0].second.x(); p_x += step_size) {
      tf2::Vector3 candidate = {p_x, p_y, 0.0};
      auto intersections = countIntersections(
        polygon,
        candidate,
        ray_cast_source);
      if ((intersections % 2) == 1) {
        for (double height = 0.0; height < walls_height_; height += 0.05) {
          auto room_point = candidate;
          room_point.setZ(height);
          semantic_map_->insertVoxel(
            room_point.x(),
            room_point.y(),
            room_point.z(),
            region,
            *regions_register_);
        }
      }
    }
  }
}
}  // namespace plugins
}  // namespace remap

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(remap::plugins::PluginNavMap, remap::plugins::PluginBase)

// 2D ray-casting
// Step 1: determine a point out of the polygon
// to do so we can compute the minimum bounding box for the polygon
// from here, compu