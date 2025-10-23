/**
 * ============================================================================
 * VaultManager
 *
 * Owns a simple waypoint "vault" directory on disk and exposes services to:
 *  - List files with basic metadata
 *  - Save current waypoints via existing save_waypoints_server
 *  - Load a file into the WaypointLooper (set params only)
 *  - Publish a one-shot preview (Waypoints) for GUI consumption
 * ====================================================================================
 *
 * Neobotix GmbH
 * Author: Adarsh Karan K P
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <neo_waypoint_follower/msg/waypoints.hpp>

#include <neo_waypoint_follower/srv/vault_list.hpp>
#include <neo_waypoint_follower/srv/vault_save_current.hpp>
#include <neo_waypoint_follower/srv/vault_load_to_looper.hpp>
#include <neo_waypoint_follower/srv/vault_preview.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <vector>
#include <cmath>

namespace fs = std::filesystem;

class VaultManager : public rclcpp::Node
{
public:
  VaultManager()
  : rclcpp::Node("vault_manager")
  {
    vault_dir_ = declare_parameter<std::string>("vault_dir", "/var/lib/neo/waypoints");
    save_server_node_name_ = declare_parameter<std::string>("save_server_node", "/save_waypoints_server");
    looper_node_name_ = declare_parameter<std::string>("looper_node", "/waypoint_looper");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    // Callback groups
    services_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    clients_cbg_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Publisher
    preview_publisher_ = create_publisher<neo_waypoint_follower::msg::Waypoints>(
      "/vault/preview_waypoints",
      rclcpp::QoS(1).transient_local().reliable()
    );

    // Services
    list_service_ = this->create_service<neo_waypoint_follower::srv::VaultList>(
      "/vault/list",
      std::bind(&VaultManager::listCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_
    );

    save_service_ = this->create_service<neo_waypoint_follower::srv::VaultSaveCurrent>(
      "/vault/save_current",
      std::bind(&VaultManager::saveCurrentCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_
    );

    load_service_ = this->create_service<neo_waypoint_follower::srv::VaultLoadToLooper>(
      "/vault/load_to_looper",
      std::bind(&VaultManager::loadToLooperCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_
    );

    preview_service_ = this->create_service<neo_waypoint_follower::srv::VaultPreview>(
      "/vault/preview_once",
      std::bind(&VaultManager::previewCallback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      services_cbg_
    );

    // Clients
    save_server_set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      save_server_node_name_ + std::string("/set_parameters"),
      rclcpp::ServicesQoS(),
      clients_cbg_);
    looper_set_params_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      looper_node_name_ + std::string("/set_parameters"),
      rclcpp::ServicesQoS(),
      clients_cbg_);
    save_server_trigger_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/save_waypoints",
      rclcpp::ServicesQoS(),
      clients_cbg_);

    ensureVaultDir_();
  }

private:
  // Validates a base filename
  // appends .yaml if it is missing
  std::string makeFilename_(const std::string &raw) const
  {
    std::string name = raw;
    // trim spaces
    name.erase(0, name.find_first_not_of(" \t\r\n"));
    name.erase(name.find_last_not_of(" \t\r\n") + 1);
    // allow only [A-Za-z0-9_.-]
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
    if (clean.empty()) clean = "waypoints";
    if (clean.size() < 5 || clean.substr(clean.size()-5) != ".yaml") {
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
        RCLCPP_WARN(get_logger(), "Failed to create vault dir %s: %s", vault_dir_.c_str(), ec.message().c_str());
      }
    }
  }

  // load a vault YAML
  bool loadWaypointsFromYaml_(const fs::path &path,
                              std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> &out) const
  {
    try {
      YAML::Node root = YAML::LoadFile(path.string());
      YAML::Node wps  = root["waypoints"];
      if (!wps || !wps.IsMap()) return false;

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

  // Computes count and checks whether first/last positions are within epsilon
  void computeStats_(const std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> &v,
                     int &points, bool &has_loop, double epsilon=0.1) const
  {
    points = static_cast<int>(v.size());
    has_loop = false;
    if (points >= 2) {
      const auto &a = v.front().second.pose.position;
      const auto &b = v.back().second.pose.position;
      const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
      const double d = std::sqrt(dx*dx + dy*dy + dz*dz);
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
      if (ps.header.frame_id.empty()) ps.header.frame_id = frame_id_;
      if (ps.header.stamp.sec == 0 && ps.header.stamp.nanosec == 0) ps.header.stamp = now();
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

    std::error_code ec;
    if (!fs::exists(vault_dir_, ec)) {
      res->success = false;
      res->message = "Vault directory does not exist";
      return;
    }

    for (const auto &entry : fs::directory_iterator(vault_dir_, ec)) {
      if (ec) break;
      if (!entry.is_regular_file()) continue;
      const auto p = entry.path();
      if (p.extension() != ".yaml") continue;

      std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> tmp;
      int count = 0; bool loop = false;
      if (loadWaypointsFromYaml_(p, tmp)) {
        computeStats_(tmp, count, loop);
      } else {
        // unreadable/invalid → count=0, loop=false
      }

      res->filenames.push_back(p.filename().string());
      res->points.push_back(count);
      res->has_loop.push_back(loop);
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

    // overwrite check
    if (!req->allow_overwrite && fs::exists(path)) {
      res->success = false;
      res->message = "File exists: " + path.string();
      return;
    }

    // set output_file on save_waypoints_server
    if (!save_server_set_params_client_->wait_for_service(std::chrono::seconds(2))) {
      res->success = false;
      res->message = "save_waypoints_server parameters service not available";
      return;
    }
    {
      auto req_set = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      req_set->parameters.emplace_back(rclcpp::Parameter("output_file", path.string()).to_parameter_msg());
      auto fut = save_server_set_params_client_->async_send_request(req_set);
      (void)fut.get(); // response will be handled on a different thread
    }

    // call /save_waypoints
    if (!save_server_trigger_client_->wait_for_service(std::chrono::seconds(2))) {
      res->success = false;
      res->message = "/save_waypoints service not available";
      return;
    }
    auto req_trig = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = save_server_trigger_client_->async_send_request(req_trig);
    auto resp = future.get();
    if (!resp || !resp->success) {
      res->success = false;
      res->message = resp ? resp->message : "No response from /save_waypoints";
      return;
    }

    res->success = true;
    res->message = "Saved to " + path.string();
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
      if (req->loop_count > 0)
        req_set->parameters.emplace_back(rclcpp::Parameter("repeat_count", static_cast<int>(req->loop_count)).to_parameter_msg());
      if (req->wait_ms >= 0)
        req_set->parameters.emplace_back(rclcpp::Parameter("wait_at_waypoint_ms", static_cast<int>(req->wait_ms)).to_parameter_msg());
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
  std::string save_server_node_name_;
  std::string looper_node_name_;
  std::string frame_id_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr services_cbg_;
  rclcpp::CallbackGroup::SharedPtr clients_cbg_;

  rclcpp::Publisher<neo_waypoint_follower::msg::Waypoints>::SharedPtr preview_publisher_;

  rclcpp::Service<neo_waypoint_follower::srv::VaultList>::SharedPtr list_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultSaveCurrent>::SharedPtr save_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultLoadToLooper>::SharedPtr load_service_;
  rclcpp::Service<neo_waypoint_follower::srv::VaultPreview>::SharedPtr preview_service_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr save_server_set_params_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr looper_set_params_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr           save_server_trigger_client_;
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
