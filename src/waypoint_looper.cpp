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
#include <neo_waypoint_follower/msg/waypoints.hpp>
using LooperMetrics = neo_waypoint_follower::msg::LooperMetrics;

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

    // Parameter change callback. can be updated anytime
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult out;
        out.successful = true;

        int    updated_wait_ms   = wait_ms_descriptor_;
        size_t updated_repeat_count = repeat_count_;

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
          }
        }

        // Update
        wait_ms_descriptor_ = updated_wait_ms;
        repeat_count_ = updated_repeat_count;
        RCLCPP_INFO(get_logger(), "Parameters updated: wait_at_waypoint_ms=%d, repeat_count=%zu",
                    wait_ms_descriptor_,
                    repeat_count_
                );

        return out;
      });
  }

private:
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

    have_last_odom_ = false;
    running_        = true;
    goal_in_flight_ = false;

    // Reset indices for loop and waypoint
    loop_idx_       = 0;
    wp_idx_         = 0;

    looper_state_ = LooperMetrics::LOOPER_RUNNING;
    resetNav2Feedback_();
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
    // Cancel timer and active goal, reset all indices and progress
    if (delay_timer_) delay_timer_->cancel();
    paused_ = false; running_ = false; goal_in_flight_ = false; starting_ = false;
    if (current_goal_) { auto future = nav_to_pose_client_->async_cancel_goal(current_goal_); (void)future; current_goal_.reset(); }
    loop_idx_ = 0; wp_idx_ = 0;
    run_distance_ = 0.0; leg_distance_ = 0.0; have_last_odom_ = false;
    looper_state_ = LooperMetrics::LOOPER_IDLE;
    resetNav2Feedback_();
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
      YAML::Node root = YAML::LoadFile(yaml_path);
      YAML::Node waypoints = root["waypoints"];
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
        } else {
          current_goal_ = gh;
        }
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
      {
        goal_in_flight_ = false;
        current_goal_.reset();

        if (!(paused_ && result.code == rclcpp_action::ResultCode::CANCELED)) {
          // keep last nav_time; zero others if desired in reset helper
        }
        publishMetrics_();

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Reached");
            ++wp_idx_;
            break;
          case rclcpp_action::ResultCode::CANCELED:
            if (paused_) return;
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            if (stop_on_fail_) { running_ = false; finish(); return; }
            else { ++wp_idx_; }
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Result code %d", (int)result.code);
            if (stop_on_fail_) { running_ = false; finish(); return; }
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
    looper_state_ = LooperMetrics::LOOPER_FINISHED;
    distance_remaining_ = 0.0;
    eta_msg_.sec = 0; eta_msg_.nanosec = 0;
    publishMetrics_();

    if (single_goal_mode_) {
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

  // limits
  static constexpr int kMaxRepeatCount = 100;
  static constexpr int kMaxWaitAtWaypointMs = 60000;

  // other members
  int wait_ms_descriptor_;
  std::string yaml_file_, frame_id_, action_name_;
  bool stop_on_fail_;

  bool running_, goal_in_flight_;
  bool paused_ = false;
  bool single_goal_mode_ = false;
  std::atomic<bool> starting_{false};

  size_t repeat_count_, loop_idx_, wp_idx_;

  inline size_t effective_repeat_count() const { return single_goal_mode_ ? 1 : repeat_count_; }
  inline int    effective_delay_ms()    const { return single_goal_mode_ ? 0 : wait_ms_descriptor_; }

  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;

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

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  start_srv_, cancel_srv_, pause_srv_, resume_srv_, publish_loaded_waypoints_srv_;
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
