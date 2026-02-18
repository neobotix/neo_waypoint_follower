/**
 * ============================================================================
 * VaultManager
 *
 * Owns a waypoint vault directory on disk and exposes services to:
 *  - List files with stats + optional metadata
 *  - Save current waypoints from MarkerArray into vault YAML
 *  - Load a file into the WaypointLooper (set params only)
 *  - Publish a one-shot preview (Waypoints) for GUI consumption
 *
 * Neobotix GmbH
 * Author: Adarsh Karan K P
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include <neo_waypoint_follower/msg/waypoints.hpp>
#include <neo_waypoint_follower/srv/vault_list.hpp>
#include <neo_waypoint_follower/srv/vault_save_current.hpp>
#include <neo_waypoint_follower/srv/vault_load_to_looper.hpp>
#include <neo_waypoint_follower/srv/vault_preview.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace fs = std::filesystem;

class VaultManager : public rclcpp::Node
{
public:
  VaultManager()
  : rclcpp::Node("vault_manager")
  {
    vault_dir_ = declare_parameter<std::string>("vault_dir", "/var/lib/neo/waypoints");
    looper_node_name_ = declare_parameter<std::string>("looper_node", "/waypoint_looper");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    waypoints_topic_ = declare_parameter<std::string>("waypoints_topic", "/waypoints");

    // Callback groups
    services_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    clients_cbg_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Subscription: latest MarkerArray used by /vault/save_current
    waypoints_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      waypoints_topic_,
      10,
      std::bind(&VaultManager::markersCallback, this, std::placeholders::_1));

    // Publisher
    preview_publisher_ = create_publisher<neo_waypoint_follower::msg::Waypoints>(
      "/vault/preview_waypoints",
      rclcpp::QoS(1).transient_local().reliable());

    // Services
    list_service_ = this->create_service<neo_waypoint_follower::srv::VaultList>(
      "/vault/list",
      std::bind(&VaultManager::listCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_);

    save_service_ = this->create_service<neo_waypoint_follower::srv::VaultSaveCurrent>(
      "/vault/save_current",
      std::bind(&VaultManager::saveCurrentCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_);

    load_service_ = this->create_service<neo_waypoint_follower::srv::VaultLoadToLooper>(
      "/vault/load_to_looper",
      std::bind(&VaultManager::loadToLooperCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_);

    preview_service_ = this->create_service<neo_waypoint_follower::srv::VaultPreview>(
      "/vault/preview_once",
      std::bind(&VaultManager::previewCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_);

    // Clients
    looper_set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      looper_node_name_ + std::string("/set_parameters"),
      rclcpp::ServicesQoS(),
      clients_cbg_);

    ensureVaultDir_();
  }

private:
  static std::string trim_(const std::string &s)
  {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
  }

  // Validates a base filename and appends .yaml if missing.
  std::string makeFilename_(const std::string &raw) const
  {
    std::string name = trim_(raw);

    std::string clean;
    clean.reserve(name.size());
    for (char c : name) {
      if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
          c == '_' || c == '-' || c == '.') {
        clean.push_back(c);
      } else {
        clean.push_back('_');
      }
    }

    if (clean.empty()) {
      clean = "waypoints";
    }
    if (clean.size() < 5 || clean.substr(clean.size() - 5) != ".yaml") {
      clean += ".yaml";
    }
    return clean;
  }

  fs::path absolutePath_(const std::string &filename) const
  {
    return fs::path(vault_dir_) / fs::path(filename);
  }

  void ensureVaultDir_()
  {
    std::error_code ec;
    if (!fs::exists(vault_dir_, ec)) {
      fs::create_directories(vault_dir_, ec);
      if (ec) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to create vault dir %s: %s",
          vault_dir_.c_str(),
          ec.message().c_str());
      }
    }
  }

  void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(markers_mutex_);
    latest_markers_ = *msg;
  }


  // Matches text markers to geometry markers by marker ID.
  // Returns entries ordered by marker ID to preserve route sequence.
  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> extractWaypointsFromMarkers_(
    const visualization_msgs::msg::MarkerArray &arr) const
  {
    std::map<int, std::string> text_by_id;
    std::map<int, geometry_msgs::msg::Pose> pose_by_id;

    for (const auto &m : arr.markers) {
      if (m.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING && !m.text.empty()) {
        text_by_id[m.id] = m.text;
      }

      if (m.type == visualization_msgs::msg::Marker::CUBE ||
          m.type == visualization_msgs::msg::Marker::SPHERE ||
          m.type == visualization_msgs::msg::Marker::CYLINDER ||
          m.type == visualization_msgs::msg::Marker::ARROW) {
        pose_by_id[m.id] = m.pose;
      }
    }

    std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> ordered;
    std::map<std::string, size_t> name_to_index;

    for (const auto &kv : text_by_id) {
      const int id = kv.first;
      const std::string &name = kv.second;
      geometry_msgs::msg::Pose pose;

      auto it = pose_by_id.find(id);
      if (it != pose_by_id.end()) {
        pose = it->second;
      } else {
        for (const auto &m : arr.markers) {
          if (m.id == id) {
            pose = m.pose;
            break;
          }
        }
      }

      const auto seen_it = name_to_index.find(name);
      if (seen_it == name_to_index.end()) {
        name_to_index[name] = ordered.size();
        ordered.emplace_back(name, pose);
      } else {
        // Keep first sequence position; update pose for duplicate names.
        ordered[seen_it->second].second = pose;
      }
    }

    return ordered;
  }

  bool writeVaultYaml_(
    const fs::path &path,
    const std::string &route_name,
    const std::string &description,
    const std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> &waypoints) const
  {
    std::ofstream fout(path);
    if (!fout.is_open()) {
      return false;
    }

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "metadata" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << route_name;
    out << YAML::Key << "description" << YAML::Value << description;
    out << YAML::EndMap;

    out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginMap;
    for (const auto &kv : waypoints) {
      const auto &name = kv.first;
      const auto &pose = kv.second;

      out << YAML::Key << name << YAML::Value << YAML::BeginMap;

      out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << pose.position.x;
      out << YAML::Key << "y" << YAML::Value << pose.position.y;
      out << YAML::Key << "z" << YAML::Value << pose.position.z;
      out << YAML::EndMap;

      out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << pose.orientation.x;
      out << YAML::Key << "y" << YAML::Value << pose.orientation.y;
      out << YAML::Key << "z" << YAML::Value << pose.orientation.z;
      out << YAML::Key << "w" << YAML::Value << pose.orientation.w;
      out << YAML::EndMap;

      out << YAML::EndMap;
    }
    out << YAML::EndMap;

    out << YAML::EndMap;

    fout << out.c_str();
    fout.close();
    return true;
  }

  // Load waypoints and optional metadata from vault YAML.
  bool loadWaypointsFromYaml_(
    const fs::path &path,
    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> &out,
    std::string *route_name = nullptr,
    std::string *description = nullptr) const
  {
    try {
      YAML::Node root = YAML::LoadFile(path.string());

      if (route_name || description) {
        YAML::Node metadata = root["metadata"];
        if (metadata && metadata.IsMap()) {
          if (route_name && metadata["name"] && metadata["name"].IsScalar()) {
            *route_name = trim_(metadata["name"].as<std::string>());
          }
          if (description && metadata["description"] && metadata["description"].IsScalar()) {
            *description = trim_(metadata["description"].as<std::string>());
          }
        }
      }

      YAML::Node wps = root["waypoints"];
      if (!wps || !wps.IsMap()) {
        return false;
      }

      out.clear();
      for (auto it = wps.begin(); it != wps.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const YAML::Node &p = it->second;

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id_;
        ps.pose.position.x = p["position"]["x"].as<double>();
        ps.pose.position.y = p["position"]["y"].as<double>();
        ps.pose.position.z = p["position"]["z"].as<double>();
        ps.pose.orientation.x = p["orientation"]["x"].as<double>();
        ps.pose.orientation.y = p["orientation"]["y"].as<double>();
        ps.pose.orientation.z = p["orientation"]["z"].as<double>();
        ps.pose.orientation.w = p["orientation"]["w"].as<double>();
        out.push_back({name, ps});
      }
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "YAML parse error for %s: %s", path.string().c_str(), e.what());
      return false;
    }
  }

  // Computes count and checks whether first/last positions are within epsilon.
  void computeStats_(
    const std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> &v,
    int &points,
    bool &has_loop,
    double epsilon = 0.1) const
  {
    points = static_cast<int>(v.size());
    has_loop = false;
    if (points >= 2) {
      const auto &a = v.front().second.pose.position;
      const auto &b = v.back().second.pose.position;
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      const double dz = a.z - b.z;
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      has_loop = (d < epsilon);
    }
  }

  neo_waypoint_follower::msg::Waypoints buildWaypointsMsg_(
    const std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> &v) const
  {
    neo_waypoint_follower::msg::Waypoints m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.names.reserve(v.size());
    m.poses.reserve(v.size());

    for (const auto &kv : v) {
      m.names.push_back(kv.first);
      auto ps = kv.second;
      if (ps.header.frame_id.empty()) {
        ps.header.frame_id = frame_id_;
      }
      if (ps.header.stamp.sec == 0 && ps.header.stamp.nanosec == 0) {
        ps.header.stamp = now();
      }
      m.poses.push_back(ps);
    }

    return m;
  }

  // /vault/list
  void listCallback(
    const std::shared_ptr<neo_waypoint_follower::srv::VaultList::Request>,
    std::shared_ptr<neo_waypoint_follower::srv::VaultList::Response> res)
  {
    res->filenames.clear();
    res->points.clear();
    res->has_loop.clear();
    res->names.clear();
    res->descriptions.clear();

    std::error_code ec;
    if (!fs::exists(vault_dir_, ec)) {
      res->success = false;
      res->message = "Vault directory does not exist";
      return;
    }

    std::vector<fs::path> yaml_files;
    for (const auto &entry : fs::directory_iterator(vault_dir_, ec)) {
      if (ec) {
        break;
      }
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto p = entry.path();
      if (p.extension() == ".yaml") {
        yaml_files.push_back(p);
      }
    }
    std::sort(yaml_files.begin(), yaml_files.end());

    for (const auto &p : yaml_files) {
      std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> tmp;
      int count = 0;
      bool loop = false;
      std::string route_name;
      std::string route_description;

      if (loadWaypointsFromYaml_(p, tmp, &route_name, &route_description)) {
        computeStats_(tmp, count, loop);
      }

      if (route_name.empty()) {
        route_name = p.stem().string();
      }

      res->filenames.push_back(p.filename().string());
      res->points.push_back(count);
      res->has_loop.push_back(loop);
      res->names.push_back(route_name);
      res->descriptions.push_back(route_description);
    }

    res->success = true;
    res->message = "OK";
  }

  // /vault/save_current
  void saveCurrentCallback(
    const std::shared_ptr<neo_waypoint_follower::srv::VaultSaveCurrent::Request> req,
    std::shared_ptr<neo_waypoint_follower::srv::VaultSaveCurrent::Response> res)
  {
    const std::string fname = makeFilename_(req->filename);
    const fs::path path = absolutePath_(fname);

    // Overwrite check
    if (!req->allow_overwrite && fs::exists(path)) {
      res->success = false;
      res->message = "File exists: " + path.string();
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    {
      std::lock_guard<std::mutex> lock(markers_mutex_);
      if (!latest_markers_) {
        res->success = false;
        res->message = "No MarkerArray received yet on " + waypoints_topic_;
        return;
      }
      markers = *latest_markers_;
    }

    auto waypoints = extractWaypointsFromMarkers_(markers);
    if (waypoints.empty()) {
      res->success = false;
      res->message = "No named waypoints found in latest MarkerArray";
      return;
    }

    std::string route_name = trim_(req->name);
    if (route_name.empty()) {
      route_name = fs::path(fname).stem().string();
    }
    const std::string description = trim_(req->description);

    if (!writeVaultYaml_(path, route_name, description, waypoints)) {
      res->success = false;
      res->message = "Failed to write YAML: " + path.string();
      return;
    }

    res->success = true;
    res->message = "Saved " + std::to_string(waypoints.size()) + " waypoints to " + path.string();
  }

  // /vault/load_to_looper
  void loadToLooperCallback(
    const std::shared_ptr<neo_waypoint_follower::srv::VaultLoadToLooper::Request> req,
    std::shared_ptr<neo_waypoint_follower::srv::VaultLoadToLooper::Response> res)
  {
    const std::string fname = makeFilename_(req->filename);
    const fs::path path = absolutePath_(fname);

    if (!fs::exists(path)) {
      res->success = false;
      res->message = "File not found: " + path.string();
      return;
    }

    if (!looper_set_params_client_->wait_for_service(std::chrono::seconds(2))) {
      res->success = false;
      res->message = "waypoint_looper parameters service not available";
      return;
    }

    auto req_set = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req_set->parameters.emplace_back(rclcpp::Parameter("yaml_file", path.string()).to_parameter_msg());
    if (req->set_loop_params) {
      if (req->loop_count > 0) {
        req_set->parameters.emplace_back(
          rclcpp::Parameter("repeat_count", static_cast<int>(req->loop_count)).to_parameter_msg());
      }
      if (req->wait_ms >= 0) {
        req_set->parameters.emplace_back(
          rclcpp::Parameter("wait_at_waypoint_ms", static_cast<int>(req->wait_ms)).to_parameter_msg());
      }
    }

    {
      auto fut = looper_set_params_client_->async_send_request(req_set);
      (void)fut.get();
    }

    res->success = true;
    res->message = "Loaded into looper: " + path.string();
  }

  // /vault/preview_once
  void previewCallback(
    const std::shared_ptr<neo_waypoint_follower::srv::VaultPreview::Request> req,
    std::shared_ptr<neo_waypoint_follower::srv::VaultPreview::Response> res)
  {
    const std::string fname = makeFilename_(req->filename);
    const fs::path path = absolutePath_(fname);

    if (!fs::exists(path)) {
      res->success = false;
      res->message = "File not found: " + path.string();
      return;
    }

    std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> v;
    if (!loadWaypointsFromYaml_(path, v)) {
      res->success = false;
      res->message = "YAML parse failed";
      return;
    }

    preview_publisher_->publish(buildWaypointsMsg_(v));
    res->success = true;
    res->message = "Published preview to /vault/preview_waypoints";
  }

private:
  std::string vault_dir_;
  std::string looper_node_name_;
  std::string frame_id_;
  std::string waypoints_topic_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr services_cbg_;
  rclcpp::CallbackGroup::SharedPtr clients_cbg_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
  std::mutex markers_mutex_;
  std::optional<visualization_msgs::msg::MarkerArray> latest_markers_;

  rclcpp::Publisher<neo_waypoint_follower::msg::Waypoints>::SharedPtr preview_publisher_;

  rclcpp::Service<neo_waypoint_follower::srv::VaultList>::SharedPtr list_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultSaveCurrent>::SharedPtr save_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultLoadToLooper>::SharedPtr load_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultPreview>::SharedPtr preview_service_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr looper_set_params_client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VaultManager>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
