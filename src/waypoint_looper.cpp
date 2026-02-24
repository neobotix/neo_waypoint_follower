/**
 * ============================================================================
 * WaypointLooper
 *
 * - Start: Begins waypoint execution from the first waypoint.
 * - Pause: Temporarily halts waypoint execution, progress is preserved.
 * - Resume: Continues execution from the paused waypoint and loop index.
 * - Cancel: Aborts the loop and resets all progress; does not auto-restart.
 * ============================================================================
 * 
 * Neobotix GmbH
 * Author: Adarsh Karan K P
 *
 * NOTE:
 * 1. Lowering repeat_count below current loop_idx_ will cause the run to finish
 *    right after the CURRENT goal completes
 * 2. Changing wait_at_waypoint_ms does not change an already running timer;
 *    it takes effect from the next waypoint
 */

#include <cmath>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <mutex>
#include <random>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <yaml-cpp/yaml.h>
#include <neo_waypoint_follower/msg/looper_metrics.hpp>
#include <neo_waypoint_follower/msg/run_history_entry.hpp>
#include <neo_waypoint_follower/msg/waypoints.hpp>
#include <neo_waypoint_follower/srv/run_history_list.hpp>
#include <neo_waypoint_follower/srv/run_history_delete.hpp>
#include <neo_waypoint_follower/srv/run_history_clear.hpp>
using LooperMetrics = neo_waypoint_follower::msg::LooperMetrics;
using RunHistoryEntryMsg = neo_waypoint_follower::msg::RunHistoryEntry;
using RunHistoryListSrv = neo_waypoint_follower::srv::RunHistoryList;
using RunHistoryDeleteSrv = neo_waypoint_follower::srv::RunHistoryDelete;
using RunHistoryClearSrv = neo_waypoint_follower::srv::RunHistoryClear;
namespace fs = std::filesystem;

class WaypointLooper : public rclcpp::Node {
public:

  WaypointLooper()
  : Node("waypoint_looper_seq"),
    running_(false), // to track if the loop is running
    goal_in_flight_(false), // to track if a goal is currently being processed
    loop_idx_(0), // current loop iteration
    wp_idx_(0) // current waypoint index
  {
    yaml_file_    = declare_parameter<std::string>("yaml_file", std::string(getenv("HOME")) + "/waypoints.yaml");
    frame_id_     = declare_parameter<std::string>("frame_id", "map");
    has_loop_param_ = declare_parameter<bool>("has_loop", false);
    
    // repeat_count descriptor
    rcl_interfaces::msg::ParameterDescriptor repeat_count_descriptor;
    repeat_count_descriptor.description = "Total number of loops for this run";
    rcl_interfaces::msg::IntegerRange repeat_count_limits;
    repeat_count_limits.from_value = 1; // lower limit of repeat count
    repeat_count_limits.to_value   = kMaxRepeatCount; // upper limit of repeat count
    repeat_count_limits.step       = 1;
    repeat_count_descriptor.integer_range = {repeat_count_limits};
    repeat_count_ = static_cast<size_t>(declare_parameter<int>("repeat_count", 100, repeat_count_descriptor));

    // wait_at_waypoint_ms descriptor
    rcl_interfaces::msg::ParameterDescriptor wait_desc;
    wait_desc.description = "Wait duration at each waypoint (ms)";
    rcl_interfaces::msg::IntegerRange wait_ms_limits;
    wait_ms_limits.from_value = 0; // lower limit of wait time
    wait_ms_limits.to_value   = kMaxWaitAtWaypointMs; // upper limit of wait time
    wait_ms_limits.step       = 1;
    wait_desc.integer_range = {wait_ms_limits};
    wait_ms_descriptor_ = declare_parameter<int>("wait_at_waypoint_ms", 200, wait_desc);

    rcl_interfaces::msg::ParameterDescriptor history_cap_desc;
    history_cap_desc.description = "Max navigation run history entries retained";
    rcl_interfaces::msg::IntegerRange history_cap_limits;
    history_cap_limits.from_value = 1;
    history_cap_limits.to_value = 200;
    history_cap_limits.step = 1;
    history_cap_desc.integer_range = {history_cap_limits};
    history_cap_ = declare_parameter<int>("history_cap", 15, history_cap_desc);
    history_dir_ = declare_parameter<std::string>("history_dir", "/var/lib/neo/lemma-gui/history");
    history_file_name_ = declare_parameter<std::string>("history_file", "navigation_runs.json");
    history_file_path_ = (fs::path(history_dir_) / history_file_name_).string();
    
    action_name_  = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    stop_on_fail_ = declare_parameter<bool>("stop_on_failure", true);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");

    // Navigate to Pose action client
    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_name_);

    // Services
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "start_waypoint_loop",
      std::bind(&WaypointLooper::startCallback, this, std::placeholders::_1, std::placeholders::_2));

    pause_srv_ = create_service<std_srvs::srv::Trigger>(
      "pause_waypoint_loop",
      std::bind(&WaypointLooper::pauseCallback, this, std::placeholders::_1, std::placeholders::_2));

    resume_srv_ = create_service<std_srvs::srv::Trigger>(
      "resume_waypoint_loop",
      std::bind(&WaypointLooper::resumeCallback, this, std::placeholders::_1, std::placeholders::_2));

    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      "cancel_waypoint_loop",
      std::bind(&WaypointLooper::cancelCallback, this, std::placeholders::_1, std::placeholders::_2));

    run_history_list_srv_ = create_service<RunHistoryListSrv>(
      "/run_history/list",
      std::bind(&WaypointLooper::runHistoryListCallback, this, std::placeholders::_1, std::placeholders::_2));

    run_history_delete_srv_ = create_service<RunHistoryDeleteSrv>(
      "/run_history/delete",
      std::bind(&WaypointLooper::runHistoryDeleteCallback, this, std::placeholders::_1, std::placeholders::_2));

    run_history_clear_srv_ = create_service<RunHistoryClearSrv>(
      "/run_history/clear",
      std::bind(&WaypointLooper::runHistoryClearCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Odom subscription for distance tracking
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){
        if (!(running_ && !paused_)) { have_last_odom_ = false; return; }
        const auto & p = msg->pose.pose.position;
        if (!have_last_odom_) { last_odom_p_ = p; have_last_odom_ = true; return; }
        double dx = p.x - last_odom_p_.x, dy = p.y - last_odom_p_.y;
        double d = std::hypot(dx, dy);
        if (d > 1e-6) { run_distance_ += d; leg_distance_ += d; last_odom_p_ = p; }
        publishMetrics_();
      });

    auto metrics_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    metrics_pub_ = create_publisher<LooperMetrics>("/waypoint_loop/metrics", metrics_qos);

    metrics_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publishMetrics_(); });

    // Publish the currently loaded waypoints
    loaded_waypoints_pub_ = create_publisher<neo_waypoint_follower::msg::Waypoints>(
      "/waypoint_loop/loaded_waypoints",
      rclcpp::QoS(1).transient_local());

    // Service to publish the loaded waypoints once
    publish_loaded_waypoints_srv_ = create_service<std_srvs::srv::Trigger>(
      "publish_loaded_waypoints",
      std::bind(&WaypointLooper::publishLoadedWaypointsCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    if (!loadRunHistoryFromDisk_()) {
      RCLCPP_WARN(
        get_logger(),
        "Run history load failed (path: %s). Starting with empty history.",
        history_file_path_.c_str());
    }

    // Parameter change callback. can be updated anytime
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult out;
        out.successful = true;

        int    updated_wait_ms   = wait_ms_descriptor_;
        size_t updated_repeat_count = repeat_count_;
        bool   updated_has_loop = has_loop_param_;
        int    updated_history_cap = history_cap_;

        // Validate both params
        for (const auto & p : params) {
          if (p.get_name() == "wait_at_waypoint_ms") {
            int v = p.as_int();
            if (v < 0 || v > kMaxWaitAtWaypointMs) {
              out.successful = false;
              out.reason = "wait_at_waypoint_ms must be in [0, " + std::to_string(kMaxWaitAtWaypointMs) + "]";
              return out;
            }
            updated_wait_ms = v;
          }

          else if (p.get_name() == "repeat_count") {
            int v = p.as_int();
            if (v < 1 || v > kMaxRepeatCount) {
              out.successful = false;
              out.reason = "repeat_count must be in [1, " + std::to_string(kMaxRepeatCount) + "]";
              return out;
            }
            updated_repeat_count = static_cast<size_t>(v);
          } else if (p.get_name() == "has_loop") {
            updated_has_loop = p.as_bool();
          } else if (p.get_name() == "history_cap") {
            int v = p.as_int();
            if (v < 1 || v > 200) {
              out.successful = false;
              out.reason = "history_cap must be in [1, 200]";
              return out;
            }
            updated_history_cap = v;
          }
        }

        // Update
        wait_ms_descriptor_ = updated_wait_ms;
        repeat_count_ = updated_repeat_count;
        has_loop_param_ = updated_has_loop;
        history_cap_ = updated_history_cap;
        trimRunHistoryToCap_();
        (void)persistRunHistoryToDisk_();
        RCLCPP_INFO(get_logger(), "Parameters updated: wait_at_waypoint_ms=%d, repeat_count=%zu, has_loop=%s, history_cap=%d",
                    wait_ms_descriptor_,
                    repeat_count_,
                    has_loop_param_ ? "true" : "false",
                    history_cap_
                );

        return out;
      });
  }

private:
  static std::string trimCopy_(const std::string &value)
  {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  }

  static std::string jsonEscape_(const std::string &value)
  {
    std::string out;
    out.reserve(value.size() + 16);
    for (unsigned char ch : value) {
      switch (ch) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
          if (ch < 0x20) {
            std::ostringstream esc;
            esc << "\\u"
                << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
                << static_cast<unsigned int>(ch);
            out += esc.str();
          } else {
            out.push_back(static_cast<char>(ch));
          }
          break;
      }
    }
    return out;
  }

  template<typename T>
  static T nodeScalarOr_(const YAML::Node &node, const T &fallback)
  {
    if (!node || !node.IsScalar()) {
      return fallback;
    }
    try {
      return node.as<T>();
    } catch (...) {
      return fallback;
    }
  }

  static bool parseBoolNode_(const YAML::Node &node, bool fallback)
  {
    if (!node || !node.IsScalar()) return fallback;
    try {
      return node.as<bool>();
    } catch (...) {
      try {
        std::string lower = trimCopy_(node.as<std::string>());
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
          return static_cast<char>(std::tolower(c));
        });
        if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") return true;
        if (lower == "false" || lower == "0" || lower == "no" || lower == "off") return false;
      } catch (...) {
        return fallback;
      }
      return fallback;
    }
  }

  uint64_t nowMs_() const
  {
    const auto now = std::chrono::system_clock::now();
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
  }

  std::string generateRunId_() const
  {
    const auto now = std::chrono::system_clock::now();
    const auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    const auto millis = ms_since_epoch.count() % 1000;
    const std::time_t t = std::chrono::system_clock::to_time_t(now);

    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif

    std::ostringstream ts;
    ts << std::put_time(&tm_utc, "%Y%m%dT%H%M%S")
       << std::setw(3) << std::setfill('0') << millis << "Z";

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> hex_dist(0, 15);
    std::ostringstream suffix;
    for (int i = 0; i < 6; ++i) {
      suffix << std::hex << std::nouppercase << hex_dist(gen);
    }

    return "run_" + ts.str() + "_" + suffix.str();
  }

  std::string deriveRouteName_() const
  {
    const std::string from_meta = trimCopy_(loaded_route_name_);
    if (!from_meta.empty()) {
      return from_meta;
    }
    if (!active_yaml_file_.empty()) {
      const fs::path p(active_yaml_file_);
      const std::string stem = trimCopy_(p.stem().string());
      if (!stem.empty()) {
        return stem;
      }
      const std::string base = trimCopy_(p.filename().string());
      if (!base.empty()) {
        return base;
      }
    }
    return "Unnamed route";
  }

  float computeCompletionPct_(const std::string &status, float traveled, float remaining) const
  {
    if (status == "SUCCEEDED") return 100.0F;
    const double denom = static_cast<double>(traveled) + static_cast<double>(remaining);
    if (denom <= 1e-6) return 0.0F;
    const double pct = (static_cast<double>(traveled) / denom) * 100.0;
    return static_cast<float>(std::max(0.0, std::min(99.9, pct)));
  }

  std::string determineFailureCauseCode_() const
  {
    auto contains_ci = [](const std::string &text, const std::string &needle) -> bool {
      auto lowered_text = text;
      auto lowered_needle = needle;
      std::transform(lowered_text.begin(), lowered_text.end(), lowered_text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      std::transform(lowered_needle.begin(), lowered_needle.end(), lowered_needle.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      return lowered_text.find(lowered_needle) != std::string::npos;
    };

    if (contains_ci(status_message_, "telemetry stale") ||
        contains_ci(status_message_, "telemetry degraded") ||
        contains_ci(status_message_, "stale telemetry") ||
        contains_ci(nav_error_msg_, "telemetry stale") ||
        contains_ci(nav_error_msg_, "telemetry degraded") ||
        contains_ci(nav_error_msg_, "stale telemetry")) {
      return "TELEMETRY_STALE";
    }
    if (status_message_.find("Goal rejected") != std::string::npos) return "GOAL_REJECTED";
    if (nav_error_code_ == 104) return "CONTROLLER_PATIENCE_EXCEEDED";
    if (nav_error_code_ == 106) return "NO_VALID_CONTROL";
    if (nav_error_code_ == 103) return "INVALID_PATH";
    if (nav_error_code_ == 105) return "FAILED_TO_MAKE_PROGRESS";
    if (nav_result_code_ == static_cast<uint8_t>(rclcpp_action::ResultCode::ABORTED)) return "NAV2_ABORTED";
    return "UNKNOWN_ERROR";
  }

  void beginRunContext_()
  {
    run_context_active_ = true;
    run_id_ = generateRunId_();
    run_started_at_ms_ = nowMs_();
    run_pause_count_ = 0;
    run_route_waypoints_ = static_cast<uint32_t>(waypoints_.size());
    run_loop_count_ = static_cast<uint32_t>(effective_repeat_count());
    run_wait_ms_ = effective_delay_ms();
    run_yaml_file_ = active_yaml_file_;
    run_route_name_ = deriveRouteName_();
    run_route_description_ = trimCopy_(loaded_route_description_);

    bool inferred_has_loop = has_loop_param_;
    if (loaded_has_loop_set_) {
      inferred_has_loop = loaded_has_loop_;
    }
    if (has_loop_param_) {
      inferred_has_loop = true;
    }
    if (effective_repeat_count() > 1) {
      inferred_has_loop = true;
    }
    run_has_loop_ = inferred_has_loop;
  }

  std::string runHistoryToJson_() const
  {
    std::ostringstream out;
    out << "{\n";
    out << "  \"schemaVersion\": 1,\n";
    out << "  \"entries\": [\n";
    for (size_t i = 0; i < run_history_entries_.size(); ++i) {
      const auto &e = run_history_entries_[i];
      out << "    {\n";
      out << "      \"run_id\": \"" << jsonEscape_(e.run_id) << "\",\n";
      out << "      \"started_at_ms\": " << e.started_at_ms << ",\n";
      out << "      \"ended_at_ms\": " << e.ended_at_ms << ",\n";
      out << "      \"status\": \"" << jsonEscape_(e.status) << "\",\n";
      out << "      \"cause_code\": \"" << jsonEscape_(e.cause_code) << "\",\n";
      out << "      \"route_name\": \"" << jsonEscape_(e.route_name) << "\",\n";
      out << "      \"route_description\": \"" << jsonEscape_(e.route_description) << "\",\n";
      out << "      \"yaml_file\": \"" << jsonEscape_(e.yaml_file) << "\",\n";
      out << "      \"route_waypoints\": " << e.route_waypoints << ",\n";
      out << "      \"has_loop\": " << (e.has_loop ? "true" : "false") << ",\n";
      out << "      \"loop_count\": " << e.loop_count << ",\n";
      out << "      \"wait_at_waypoint_ms\": " << e.wait_at_waypoint_ms << ",\n";
      out << "      \"pauses\": " << e.pauses << ",\n";
      out << "      \"wall_time_sec\": " << e.wall_time_sec << ",\n";
      out << "      \"distance_traveled\": " << e.distance_traveled << ",\n";
      out << "      \"distance_remaining\": " << e.distance_remaining << ",\n";
      out << "      \"completion_pct\": " << e.completion_pct << ",\n";
      out << "      \"nav_result_code\": " << static_cast<unsigned int>(e.nav_result_code) << ",\n";
      out << "      \"nav_error_code\": " << e.nav_error_code << ",\n";
      out << "      \"nav_error_msg\": \"" << jsonEscape_(e.nav_error_msg) << "\",\n";
      out << "      \"status_message\": \"" << jsonEscape_(e.status_message) << "\"\n";
      out << "    }" << (i + 1 < run_history_entries_.size() ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    return out.str();
  }

  bool persistRunHistoryToDisk_()
  {
    std::lock_guard<std::mutex> lock(run_history_mutex_);
    try {
      fs::create_directories(history_dir_);
      const fs::path target(history_file_path_);
      const fs::path temp = target.string() + ".tmp";
      std::ofstream fout(temp, std::ios::out | std::ios::trunc);
      if (!fout.is_open()) {
        return false;
      }
      fout << runHistoryToJson_();
      fout.close();
      if (!fout) {
        return false;
      }
      std::error_code ec;
      fs::rename(temp, target, ec);
      if (ec) {
        fs::remove(temp, ec);
        return false;
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  void trimRunHistoryToCap_()
  {
    std::lock_guard<std::mutex> lock(run_history_mutex_);
    if (history_cap_ <= 0) return;
    const size_t cap = static_cast<size_t>(history_cap_);
    if (run_history_entries_.size() > cap) {
      run_history_entries_.resize(cap);
    }
  }

  bool loadRunHistoryFromDisk_()
  {
    std::lock_guard<std::mutex> lock(run_history_mutex_);
    run_history_entries_.clear();
    try {
      const fs::path p(history_file_path_);
      if (!fs::exists(p)) {
        return true;
      }
      YAML::Node root = YAML::LoadFile(p.string());
      const int schema_version = nodeScalarOr_(root["schemaVersion"], 0);
      if (schema_version != 1) {
        RCLCPP_WARN(get_logger(), "Unsupported run history schemaVersion=%d", schema_version);
        return false;
      }
      const YAML::Node entries = root["entries"];
      if (!entries || !entries.IsSequence()) {
        return true;
      }
      run_history_entries_.reserve(entries.size());
      for (const auto &item : entries) {
        RunHistoryEntryMsg e;
        e.run_id = nodeScalarOr_(item["run_id"], std::string());
        if (e.run_id.empty()) continue;
        e.started_at_ms = nodeScalarOr_(item["started_at_ms"], static_cast<uint64_t>(0));
        e.ended_at_ms = nodeScalarOr_(item["ended_at_ms"], static_cast<uint64_t>(0));
        e.status = nodeScalarOr_(item["status"], std::string("FAILED"));
        e.cause_code = nodeScalarOr_(item["cause_code"], std::string("UNKNOWN_ERROR"));
        e.route_name = nodeScalarOr_(item["route_name"], std::string("Unnamed route"));
        e.route_description = nodeScalarOr_(item["route_description"], std::string());
        e.yaml_file = nodeScalarOr_(item["yaml_file"], std::string());
        e.route_waypoints = nodeScalarOr_(item["route_waypoints"], static_cast<uint32_t>(0));
        e.has_loop = parseBoolNode_(item["has_loop"], false);
        e.loop_count = nodeScalarOr_(item["loop_count"], static_cast<uint32_t>(1));
        e.wait_at_waypoint_ms = nodeScalarOr_(item["wait_at_waypoint_ms"], static_cast<int32_t>(0));
        e.pauses = nodeScalarOr_(item["pauses"], static_cast<uint32_t>(0));
        e.wall_time_sec = nodeScalarOr_(item["wall_time_sec"], 0.0F);
        e.distance_traveled = nodeScalarOr_(item["distance_traveled"], 0.0F);
        e.distance_remaining = nodeScalarOr_(item["distance_remaining"], 0.0F);
        e.completion_pct = nodeScalarOr_(item["completion_pct"], 0.0F);
        e.nav_result_code = nodeScalarOr_(item["nav_result_code"], static_cast<uint8_t>(0));
        e.nav_error_code = nodeScalarOr_(item["nav_error_code"], static_cast<uint16_t>(0));
        e.nav_error_msg = nodeScalarOr_(item["nav_error_msg"], std::string());
        e.status_message = nodeScalarOr_(item["status_message"], std::string());
        run_history_entries_.push_back(e);
      }
      if (history_cap_ > 0 && run_history_entries_.size() > static_cast<size_t>(history_cap_)) {
        run_history_entries_.resize(static_cast<size_t>(history_cap_));
      }
      return true;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to load run history: %s", e.what());
      run_history_entries_.clear();
      return false;
    }
  }

  void finalizeRunHistory_(const std::string &status, const std::string &cause_code)
  {
    if (!run_context_active_) return;

    const uint64_t ended_at_ms = nowMs_();
    const uint64_t elapsed_ms = ended_at_ms >= run_started_at_ms_
      ? (ended_at_ms - run_started_at_ms_)
      : 0;
    const float wall_time_sec = static_cast<float>(elapsed_ms) / 1000.0F;
    const float traveled = static_cast<float>(std::max(0.0, run_distance_));
    const float remaining = static_cast<float>(std::max(0.0, distance_remaining_));

    RunHistoryEntryMsg entry;
    entry.run_id = run_id_.empty() ? generateRunId_() : run_id_;
    entry.started_at_ms = run_started_at_ms_;
    entry.ended_at_ms = ended_at_ms;
    entry.status = status;
    entry.cause_code = cause_code;
    entry.route_name = run_route_name_.empty() ? "Unnamed route" : run_route_name_;
    entry.route_description = run_route_description_;
    entry.yaml_file = run_yaml_file_;
    entry.route_waypoints = run_route_waypoints_;
    entry.has_loop = run_has_loop_;
    entry.loop_count = run_loop_count_;
    entry.wait_at_waypoint_ms = run_wait_ms_;
    entry.pauses = run_pause_count_;
    entry.wall_time_sec = wall_time_sec;
    entry.distance_traveled = traveled;
    entry.distance_remaining = remaining;
    entry.completion_pct = computeCompletionPct_(status, traveled, remaining);
    entry.nav_result_code = nav_result_code_;
    entry.nav_error_code = nav_error_code_;
    entry.nav_error_msg = nav_error_msg_;
    entry.status_message = status_message_;

    {
      std::lock_guard<std::mutex> lock(run_history_mutex_);
      run_history_entries_.insert(run_history_entries_.begin(), entry);
      if (history_cap_ > 0 && run_history_entries_.size() > static_cast<size_t>(history_cap_)) {
        run_history_entries_.resize(static_cast<size_t>(history_cap_));
      }
    }

    if (!persistRunHistoryToDisk_()) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to persist run history (status=%s, run_id=%s) to %s",
        status.c_str(),
        entry.run_id.c_str(),
        history_file_path_.c_str());
    }

    run_context_active_ = false;
    run_started_at_ms_ = 0;
    run_pause_count_ = 0;
    run_route_waypoints_ = 0;
    run_has_loop_ = false;
    run_loop_count_ = 1;
    run_wait_ms_ = 0;
    run_yaml_file_.clear();
    run_route_name_.clear();
    run_route_description_.clear();
    run_id_.clear();
  }

  void runHistoryListCallback(
    const std::shared_ptr<RunHistoryListSrv::Request> /*req*/,
    std::shared_ptr<RunHistoryListSrv::Response> res)
  {
    std::lock_guard<std::mutex> lock(run_history_mutex_);
    res->success = true;
    res->message = "OK";
    res->cap = static_cast<uint32_t>(std::max(0, history_cap_));
    res->entries = run_history_entries_;
  }

  void runHistoryDeleteCallback(
    const std::shared_ptr<RunHistoryDeleteSrv::Request> req,
    std::shared_ptr<RunHistoryDeleteSrv::Response> res)
  {
    const std::string id = trimCopy_(req->run_id);
    if (id.empty()) {
      res->success = false;
      res->message = "run_id is required";
      return;
    }

    bool deleted = false;
    {
      std::lock_guard<std::mutex> lock(run_history_mutex_);
      const auto it = std::find_if(run_history_entries_.begin(), run_history_entries_.end(),
        [&id](const RunHistoryEntryMsg &e) { return e.run_id == id; });
      if (it != run_history_entries_.end()) {
        run_history_entries_.erase(it);
        deleted = true;
      }
    }

    if (!deleted) {
      res->success = false;
      res->message = "run_id not found";
      return;
    }

    if (!persistRunHistoryToDisk_()) {
      res->success = false;
      res->message = "Deleted in memory but failed to persist";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = "Deleted";
  }

  void runHistoryClearCallback(
    const std::shared_ptr<RunHistoryClearSrv::Request> /*req*/,
    std::shared_ptr<RunHistoryClearSrv::Response> res)
  {
    uint32_t deleted_count = 0;
    {
      std::lock_guard<std::mutex> lock(run_history_mutex_);
      deleted_count = static_cast<uint32_t>(run_history_entries_.size());
      run_history_entries_.clear();
    }

    if (!persistRunHistoryToDisk_()) {
      res->success = false;
      res->message = "Cleared in memory but failed to persist";
      res->deleted_count = deleted_count;
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = "Cleared";
    res->deleted_count = deleted_count;
  }

  // services
  /**
   * @brief Service callback to start waypoint loop execution.
   *
   * Resets indices, loads waypoints, and begins looping from the first waypoint.
   * Handles single-goal mode detection and action server readiness.
   *
   * @param req Service request (unused)
   * @param res Service response
   */
  void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (starting_.exchange(true)) { res->success = false; res->message = "Start already in progress"; return; }
    if (running_ || goal_in_flight_) { starting_ = false; res->success = false; res->message = "Already running; pause or cancel first"; return; }
    if (!loadYaml() || waypoints_.empty()) { starting_ = false; res->success = false; res->message = "No poses in YAML"; return; }
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      starting_ = false; res->success = false; res->message = "NavigateToPose server not up"; return;
    }

    // Detect single-goal mode (only one waypoint in YAML)
    single_goal_mode_ = (waypoints_.size() == 1);

    // clear flags/indices
    paused_ = false;
    if (delay_timer_) delay_timer_->cancel();

    // Reset odometry and progress tracking
    run_distance_ = 0.0;
    leg_distance_ = 0.0;
    run_context_active_ = false;
    run_pause_count_ = 0;

    have_last_odom_ = false;
    running_        = true;
    goal_in_flight_ = false;

    // Reset indices for loop and waypoint
    loop_idx_       = 0;
    wp_idx_         = 0;

    looper_state_ = LooperMetrics::LOOPER_RUNNING;
    status_message_ = single_goal_mode_ ? "Single-goal run started" : "Waypoint loop started";
    resetNav2Feedback_();
    resetNav2Result_();
    beginRunContext_();
    publishMetrics_();
    sendNext();
    starting_ = false;
    res->success = true;
    res->message = single_goal_mode_ ? "Started Single Goal Mode" : "Started Waypoint Loop Mode";
    RCLCPP_INFO(get_logger(), "%s started.",
                single_goal_mode_ ? "Single-goal navigation" : "Waypoint loop");
  }

  /**
   * @brief Service callback to pause waypoint loop execution.
   *
   * Cancels any active goal and preserves progress for resuming.
   *
   * @param req Service request (unused)
   * @param res Service response
   */
  void pauseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) { res->success=false; res->message="Not running"; return; }
    if (paused_)   { res->success=false; res->message="Already paused"; return; }

    paused_ = true;
    // Cancel timer and active goal if present
    if (delay_timer_) delay_timer_->cancel();
    if (current_goal_) { auto future = nav_to_pose_client_->async_cancel_goal(current_goal_); (void)future; }
    looper_state_ = LooperMetrics::LOOPER_PAUSED;
    status_message_ = "Paused by user";
    if (run_context_active_) {
      ++run_pause_count_;
    }
    res->success = true; res->message = "Paused";
    publishMetrics_();
  }

  /**
   * @brief Service callback to resume waypoint loop execution.
   *
   * Continues execution from the paused waypoint and loop index.
   *
   * @param req Service request (unused)
   * @param res Service response
   */
  void resumeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) { res->success=false; res->message="Not running"; return; }
    if (!paused_)  { res->success=false; res->message="Not paused"; return; }

    paused_ = false;
    looper_state_ = LooperMetrics::LOOPER_RUNNING;
    status_message_ = "Resumed";
    // Resume navigation if no goal is in flight
    if (!goal_in_flight_) sendNext();
    res->success = true; res->message = "Resumed";
    publishMetrics_();
  }

  /**
   * @brief Service callback to cancel waypoint loop execution.
   *
   * Aborts the loop, cancels any active goal, and resets all progress.
   *
   * @param req Service request (unused)
   * @param res Service response
   */
  void cancelCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    status_message_ = "Canceled and reset";
    finalizeRunHistory_("CANCELLED", "USER_CANCELLED");

    // Cancel timer and active goal, reset all indices and progress
    if (delay_timer_) delay_timer_->cancel();
    paused_ = false; running_ = false; goal_in_flight_ = false; starting_ = false;
    if (current_goal_) { auto future = nav_to_pose_client_->async_cancel_goal(current_goal_); (void)future; current_goal_.reset(); }
    loop_idx_ = 0; wp_idx_ = 0;
    run_distance_ = 0.0; leg_distance_ = 0.0; have_last_odom_ = false;
    looper_state_ = LooperMetrics::LOOPER_IDLE;
    resetNav2Feedback_();
    resetNav2Result_();
    res->success = true; res->message = "Canceled and reset.";
    RCLCPP_INFO(get_logger(), "Waypoint loop canceled and reset.");
    publishMetrics_();
  }

  /**
   * @brief Load waypoints from YAML file.
   *
   * Parses the configured YAML file and loads waypoints into memory.
   * Each waypoint must have position and orientation fields.
   *
   * @return True if waypoints loaded successfully, false otherwise.
   */
  bool loadYaml() {
    try {
      std::string yaml_path = yaml_file_;
      // use the updated value if file path has been changed at runtime
      (void)this->get_parameter("yaml_file", yaml_path);
      active_yaml_file_ = yaml_path;
      YAML::Node root = YAML::LoadFile(yaml_path);
      YAML::Node waypoints = root["waypoints"];
      if (!waypoints || !waypoints.IsMap()) {
        RCLCPP_ERROR(get_logger(), "YAML has no 'waypoints' map: %s", yaml_path.c_str());
        return false;
      }

      loaded_route_name_.clear();
      loaded_route_description_.clear();
      loaded_has_loop_ = false;
      loaded_has_loop_set_ = false;
      YAML::Node metadata = root["metadata"];
      if (metadata && metadata.IsMap()) {
        loaded_route_name_ = trimCopy_(nodeScalarOr_(metadata["name"], std::string()));
        loaded_route_description_ = trimCopy_(nodeScalarOr_(metadata["description"], std::string()));
        if (metadata["has_loop"] && metadata["has_loop"].IsScalar()) {
          loaded_has_loop_ = parseBoolNode_(metadata["has_loop"], false);
          loaded_has_loop_set_ = true;
        }
      }

      waypoints_.clear();

      // loop through waypoints
      for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const YAML::Node & p = it->second;
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = frame_id_;
        pose_stamped.pose.position.x = p["position"]["x"].as<double>();
        pose_stamped.pose.position.y = p["position"]["y"].as<double>();
        pose_stamped.pose.position.z = p["position"]["z"].as<double>();
        pose_stamped.pose.orientation.x = p["orientation"]["x"].as<double>();
        pose_stamped.pose.orientation.y = p["orientation"]["y"].as<double>();
        pose_stamped.pose.orientation.z = p["orientation"]["z"].as<double>();
        pose_stamped.pose.orientation.w = p["orientation"]["w"].as<double>();
        waypoints_.push_back({name, pose_stamped});
      }
      return true;
    } 
    catch (const YAML::BadFile & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open YAML file: %s", e.what());
      return false;
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
      return false;
    }
  }

  neo_waypoint_follower::msg::Waypoints buildLoadedWaypointsMsg_()
  {
    neo_waypoint_follower::msg::Waypoints msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // pre-allocate room for exactly the waypoints we have for efficiency
    msg.names.reserve(waypoints_.size());
    msg.poses.reserve(waypoints_.size());

    for (const auto & pair : waypoints_) {
      msg.names.push_back(pair.first);

      auto ps = pair.second;
      if (ps.header.frame_id.empty()) {
        ps.header.frame_id = frame_id_;
      }
      if (ps.header.stamp.sec == 0 && ps.header.stamp.nanosec == 0) {
        ps.header.stamp = now();
      }
      msg.poses.push_back(ps);
    }
    return msg;
  }

  void publishLoadedWaypointsCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!loaded_waypoints_pub_) {
      res->success = false;
      res->message = "Publisher not available";
      return;
    }

    // Reload from YAML so updates on disk are reflected here
    (void)loadYaml();

    if (waypoints_.empty()) {
      res->success = false;
      res->message = "No waypoints loaded (YAML empty or load failed)";
      return;
    }

    loaded_waypoints_pub_->publish(buildLoadedWaypointsMsg_());
    res->success = true;
    res->message = "Published loaded waypoints";
  }

  /**
   * @brief Send the next waypoint goal to the navigation action server.
   *
   * Handles sequencing, looping, and delay between waypoints.
   * Manages single-goal mode, feedback, and result handling.
   */
  void sendNext()
  {
    if (!running_ || goal_in_flight_ || paused_) return;

    // If all loops are complete, finish the run
    if (loop_idx_ >= effective_repeat_count()) { finish(); return; }

    if (wp_idx_ >= waypoints_.size()) {
      wp_idx_ = 0;
      ++loop_idx_;
      if (loop_idx_ >= effective_repeat_count()) { finish(); return; }
    }

    const auto & pair = waypoints_[wp_idx_];
    current_waypoint_name_ = pair.first;
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose = pair.second;
    goal.pose.header.stamp = now();

    if (single_goal_mode_) {
      RCLCPP_INFO(get_logger(), "Goal 1/1 -> %s", pair.first.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loop %zu/%zu -> %s (%zu/%zu)",
                  loop_idx_ + 1, effective_repeat_count(), pair.first.c_str(),
                  wp_idx_ + 1, waypoints_.size());
    }

    leg_distance_ = 0.0; have_last_odom_ = false;
    resetNav2Feedback_();
    status_message_ = std::string("Navigating to ") + current_waypoint_name_;
    looper_state_ = LooperMetrics::LOOPER_RUNNING;
    publishMetrics_();

    goal_in_flight_ = true;

    auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
             const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> fb)
      {
        eta_msg_       = fb->estimated_time_remaining;
        nav_time_msg_  = fb->navigation_time;
        distance_remaining_ = fb->distance_remaining;
        publishMetrics_();
      };

    options.goal_response_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> gh){
        if (!gh) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected");
          goal_in_flight_ = false;
          running_ = false;
          looper_state_ = LooperMetrics::LOOPER_ERROR;
          nav_result_code_ = static_cast<uint8_t>(rclcpp_action::ResultCode::UNKNOWN);
          nav_error_code_ = nav2_msgs::action::NavigateToPose::Result::UNKNOWN;
          nav_error_msg_ = "Goal rejected by action server";
          status_message_ = "Goal rejected by action server";
          finish();
        } else {
          current_goal_ = gh;
        }
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
      {
        goal_in_flight_ = false;
        current_goal_.reset();
        nav_result_code_ = static_cast<uint8_t>(result.code);
        if (result.result) {
          nav_error_code_ = result.result->error_code;
          nav_error_msg_ = result.result->error_msg;
        } else {
          nav_error_code_ = nav2_msgs::action::NavigateToPose::Result::UNKNOWN;
          nav_error_msg_.clear();
        }

        if (!(paused_ && result.code == rclcpp_action::ResultCode::CANCELED)) {
          // keep last nav_time; zero others if desired in reset helper
        }
        publishMetrics_();

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Reached");
            status_message_ = std::string("Reached ") + current_waypoint_name_;
            ++wp_idx_;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            if (paused_) {
              status_message_ = "Paused by user";
              return;
            }
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            status_message_ = nav_error_msg_.empty() ? "Goal canceled by Nav2" : nav_error_msg_;
            if (stop_on_fail_) {
              looper_state_ = LooperMetrics::LOOPER_ERROR;
              running_ = false;
              finish();
              return;
            }
            else { ++wp_idx_; }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Goal aborted");
            status_message_ = nav_error_msg_.empty() ? "Goal aborted by Nav2" : nav_error_msg_;
            if (stop_on_fail_) {
              looper_state_ = LooperMetrics::LOOPER_ERROR;
              running_ = false;
              finish();
              return;
            }
            else { ++wp_idx_; }
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Result code %d", (int)result.code);
            status_message_ = nav_error_msg_.empty() ? "Navigation failed" : nav_error_msg_;
            if (stop_on_fail_) {
              looper_state_ = LooperMetrics::LOOPER_ERROR;
              running_ = false;
              finish();
              return;
            }
            else { ++wp_idx_; }
            break;
        }

        if (!running_) return;

        const int wait_ms = effective_delay_ms();

        const bool last_goal_overall =
          (wp_idx_ >= waypoints_.size()) &&
          ((loop_idx_ + 1) >= effective_repeat_count());

        if (last_goal_overall) { finish(); return; }

        if (wait_ms > 0) {
          if (delay_timer_) delay_timer_->cancel();
          looper_state_ = LooperMetrics::LOOPER_WAITING;
          status_message_ = std::string("Waiting at waypoint ") + current_waypoint_name_;
          publishMetrics_();
          delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(wait_ms),
            [this]() {
              delay_timer_->cancel();
              if (!running_ || paused_) return;
              sendNext();
            });
        } else {
          sendNext();
        }
      };

    nav_to_pose_client_->async_send_goal(goal, options);
  }

  /**
   * @brief Complete the waypoint loop run and publish final metrics.
   *
   * Sets state to finished and logs completion.
   */
  void finish()
  {
    running_ = false;
    goal_in_flight_ = false;
    const bool has_error = looper_state_ == LooperMetrics::LOOPER_ERROR;
    if (!has_error) {
      looper_state_ = LooperMetrics::LOOPER_FINISHED;
      status_message_ = single_goal_mode_ ? "Single goal complete" : "Waypoint loop complete";
      distance_remaining_ = 0.0;
    } else if (status_message_.empty()) {
      status_message_ = "Waypoint loop failed";
    }
    eta_msg_.sec = 0; eta_msg_.nanosec = 0;
    finalizeRunHistory_(has_error ? "FAILED" : "SUCCEEDED", has_error ? determineFailureCauseCode_() : "ROUTE_COMPLETED");
    publishMetrics_();

    if (has_error) {
      RCLCPP_WARN(get_logger(), "Run finished with error: %s", status_message_.c_str());
    } else if (single_goal_mode_) {
      RCLCPP_INFO(get_logger(), "Single goal complete.");
    } else {
      RCLCPP_INFO(get_logger(), "Completed %zu loop(s).", repeat_count_);
    }
  }

  /**
   * @brief Publish current looper metrics to topic.
   *
   * Includes loop/waypoint indices, distances, ETA, and state.
   */
  void publishMetrics_()
  {
    if (!metrics_pub_) { return; }
    LooperMetrics m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;

    m.loop_idx = static_cast<uint32_t>(loop_idx_);
    m.wp_idx   = static_cast<uint32_t>(wp_idx_);
    m.current_waypoint_name = current_waypoint_name_;

    m.distance_from_start = run_distance_;
    m.distance_from_last  = leg_distance_;
    m.distance_remaining  = distance_remaining_;

    m.eta            = eta_msg_;
    m.navigation_time = nav_time_msg_;

    m.looper_state = looper_state_;
    m.nav_result_code = nav_result_code_;
    m.nav_error_code = nav_error_code_;
    m.nav_error_msg = nav_error_msg_;
    m.status_message = status_message_;
    metrics_pub_->publish(m);
  }

  /**
   * @brief Reset navigation feedback fields (distance, ETA, time).
   */
  void resetNav2Feedback_()
  {
    distance_remaining_ = 0.0;
    eta_msg_.sec = 0;     eta_msg_.nanosec = 0;
    nav_time_msg_.sec = 0; nav_time_msg_.nanosec = 0;
  }

  /**
   * @brief Reset last action result fields carried in metrics.
   */
  void resetNav2Result_()
  {
    nav_result_code_ = static_cast<uint8_t>(rclcpp_action::ResultCode::UNKNOWN);
    nav_error_code_ = nav2_msgs::action::NavigateToPose::Result::NONE;
    nav_error_msg_.clear();
  }

  // limits
  static constexpr int kMaxRepeatCount = 100;
  static constexpr int kMaxWaitAtWaypointMs = 60000;

  // other members
  int wait_ms_descriptor_;
  int history_cap_ = 15;
  std::string yaml_file_, frame_id_, action_name_;
  std::string history_dir_;
  std::string history_file_name_;
  std::string history_file_path_;
  bool stop_on_fail_;
  bool has_loop_param_ = false;

  bool running_, goal_in_flight_;
  bool paused_ = false;
  bool single_goal_mode_ = false;
  std::atomic<bool> starting_{false};

  size_t repeat_count_, loop_idx_, wp_idx_;

  inline size_t effective_repeat_count() const { return single_goal_mode_ ? 1 : repeat_count_; }
  inline int    effective_delay_ms()    const { return single_goal_mode_ ? 0 : wait_ms_descriptor_; }

  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;
  std::string active_yaml_file_;
  std::string loaded_route_name_;
  std::string loaded_route_description_;
  bool loaded_has_loop_ = false;
  bool loaded_has_loop_set_ = false;

  bool run_context_active_ = false;
  uint64_t run_started_at_ms_ = 0;
  uint32_t run_pause_count_ = 0;
  uint32_t run_route_waypoints_ = 0;
  bool run_has_loop_ = false;
  uint32_t run_loop_count_ = 1;
  int32_t run_wait_ms_ = 0;
  std::string run_yaml_file_;
  std::string run_route_name_;
  std::string run_route_description_;
  std::string run_id_;

  mutable std::mutex run_history_mutex_;
  std::vector<RunHistoryEntryMsg> run_history_entries_;

  bool have_last_odom_ = false;
  geometry_msgs::msg::Point last_odom_p_{};
  double run_distance_ = 0.0;
  double leg_distance_ = 0.0;

  std::string odom_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<LooperMetrics>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr metrics_timer_;
  rclcpp::Publisher<neo_waypoint_follower::msg::Waypoints>::SharedPtr loaded_waypoints_pub_;

  uint8_t looper_state_ = LooperMetrics::LOOPER_IDLE;
  builtin_interfaces::msg::Duration eta_msg_{};
  builtin_interfaces::msg::Duration nav_time_msg_{};
  double distance_remaining_ = 0.0;
  std::string current_waypoint_name_;
  uint8_t nav_result_code_ = static_cast<uint8_t>(rclcpp_action::ResultCode::UNKNOWN);
  uint16_t nav_error_code_ = nav2_msgs::action::NavigateToPose::Result::NONE;
  std::string nav_error_msg_;
  std::string status_message_ = "Idle";

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  start_srv_, cancel_srv_, pause_srv_, resume_srv_, publish_loaded_waypoints_srv_;
  rclcpp::Service<RunHistoryListSrv>::SharedPtr run_history_list_srv_;
  rclcpp::Service<RunHistoryDeleteSrv>::SharedPtr run_history_delete_srv_;
  rclcpp::Service<RunHistoryClearSrv>::SharedPtr run_history_clear_srv_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

/**
 * @brief Main entry point for the WaypointLooper node.
 * Initializes ROS2, spins the node, and shuts down cleanly.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointLooper>());
  rclcpp::shutdown();
  return 0;
}
