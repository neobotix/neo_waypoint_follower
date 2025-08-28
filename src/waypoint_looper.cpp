/**
 * ============================================================================
 * WaypointLooper
 *
 * - Start: Begins waypoint execution from the first waypoint.
 * - Pause: Temporarily halts waypoint execution, progress is preserved.
 * - Resume: Continues execution from the paused waypoint and loop index.
 * - Cancel: Aborts the loop and resets all progress; does not auto-restart.
 * ============================================================================
 * Neobotix GmbH
 * Author: Adarsh Karan K P
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

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
    repeat_count_ = static_cast<size_t>(declare_parameter<int>("repeat_count", 100)); //casted to to avoid signed/unsigned comparison warnings
    delay_ms_     = declare_parameter<int>("wait_at_waypoint_ms", 200);
    action_name_  = declare_parameter<std::string>("action_name", "/navigate_to_pose");
    stop_on_fail_ = declare_parameter<bool>("stop_on_failure", true);

    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, action_name_);

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
  }

private:
  // services
  /// \brief Service callback to start waypoint loop execution.
  ///
  /// Resets indices, loads waypoints, and begins looping from the first waypoint.
  ///
  /// \param req Service request (unused)
  /// \param res Service response
  void startCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (running_) { res->success = false; res->message = "Already running"; return; }
    if (!loadYaml() || waypoints_.empty()) { res->success = false; res->message = "No poses in YAML"; return; }
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      res->success = false; res->message = "NavigateToPose server not up"; return;
    }

    // clear flags/indices
    paused_ = false;
    if (delay_timer_) delay_timer_->cancel();

    running_        = true;
    goal_in_flight_ = false;
    loop_idx_       = 0;
    wp_idx_         = 0;

    sendNext();
    res->success = true;
    res->message = "Started";
    RCLCPP_INFO(get_logger(), "Waypoint looping started.");
  }

  /// \brief Service callback to pause waypoint loop execution.
  ///
  /// Cancels any active goal and preserves progress for resuming.
  ///
  /// \param req Service request (unused)
  /// \param res Service response
  void pauseCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) { res->success=false; res->message="Not running"; return; }
    if (paused_)   { res->success=false; res->message="Already paused"; return; }

    paused_ = true;
    if (delay_timer_) delay_timer_->cancel();
    if (current_goal_) {
      auto fut = client_->async_cancel_goal(current_goal_); (void)fut;
    }
    res->success = true;
    res->message = "Paused";
  }

  /// \brief Service callback to resume waypoint loop execution.
  ///
  /// Continues execution from the paused waypoint and loop index.
  ///
  /// \param req Service request (unused)
  /// \param res Service response
  void resumeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) { res->success=false; res->message="Not running"; return; }
    if (!paused_)  { res->success=false; res->message="Not paused"; return; }

    paused_ = false;
    // Re-send the current waypoint if nothing is in flight
    if (!goal_in_flight_) sendNext();
    res->success = true;
    res->message = "Resumed";
  }

  /// \brief Service callback to cancel waypoint loop execution.
  ///
  /// Aborts the loop, cancels any active goal, and resets all progress.
  ///
  /// \param req Service request (unused)
  /// \param res Service response
  void cancelCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (delay_timer_) delay_timer_->cancel();
    paused_ = false;
    running_ = false;
    goal_in_flight_ = false;

    if (current_goal_) {
      auto fut = client_->async_cancel_goal(current_goal_); (void)fut;
      current_goal_.reset();
    }

    loop_idx_ = 0;
    wp_idx_ = 0;

    res->success = true;
    res->message = "Canceled and reset.";
    RCLCPP_INFO(get_logger(), "Waypoint loop canceled and reset.");
  }

  // yaml
  /// \brief Load waypoints from YAML file.
  ///
  /// Parses the configured YAML file and loads waypoints into memory.
  ///
  /// \return True if waypoints loaded successfully, false otherwise.
  bool loadYaml() {
    try {
      YAML::Node root = YAML::LoadFile(yaml_file_);
      YAML::Node wps = root["waypoints"];
      waypoints_.clear();

      // loop through waypoints
      for (auto it = wps.begin(); it != wps.end(); ++it) {
        const std::string name = it->first.as<std::string>();
        const YAML::Node & p = it->second;
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id_;
        ps.pose.position.x = p["position"]["x"].as<double>();
        ps.pose.position.y = p["position"]["y"].as<double>();
        ps.pose.position.z = p["position"]["z"].as<double>();
        ps.pose.orientation.x = p["orientation"]["x"].as<double>();
        ps.pose.orientation.y = p["orientation"]["y"].as<double>();
        ps.pose.orientation.z = p["orientation"]["z"].as<double>();
        ps.pose.orientation.w = p["orientation"]["w"].as<double>();
        waypoints_.push_back({name, ps});
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

  // action sequencing
  /// \brief Send the next waypoint goal to the navigation action server.
  ///
  /// Handles sequencing, looping, and delay between waypoints.
  void sendNext()
  {
    if (!running_ || goal_in_flight_ || paused_) return;

    if (loop_idx_ >= repeat_count_) { finish(); return; }

    if (wp_idx_ >= waypoints_.size()) {
      wp_idx_ = 0;
      ++loop_idx_;
      if (loop_idx_ >= repeat_count_) { finish(); return; }
    }

    const auto & pair = waypoints_[wp_idx_];
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose = pair.second;
    goal.pose.header.stamp = now();

    RCLCPP_INFO(get_logger(), "Loop %zu/%zu -> %s (%zu/%zu)",
                loop_idx_ + 1, repeat_count_, pair.first.c_str(),
                wp_idx_ + 1, waypoints_.size());

    goal_in_flight_ = true;

    auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
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

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Reached");
          ++wp_idx_;  // advance on success
          break;

        case rclcpp_action::ResultCode::CANCELED:
          if (paused_) {
            // user paused, so stay at same wp_idx_
            return;
          } else {
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            if (stop_on_fail_) { running_ = false; finish(); return; }
            else { ++wp_idx_; }  // advance if not stopping on fail
          }
          break;

        default:
          RCLCPP_WARN(this->get_logger(), "Result code %d", (int)result.code);
          if (stop_on_fail_) { running_ = false; finish(); return; }
          else { ++wp_idx_; }
          break;
      }

        if (!running_) return;

        if (delay_ms_ > 0) {
          if (delay_timer_) delay_timer_->cancel();
          delay_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delay_ms_),
            [this]() {
            delay_timer_->cancel();
            if (!running_ || paused_) return;
            sendNext();
            });
        } else {
          sendNext();
        }
      };

    client_->async_send_goal(goal, options);
  }

  /// \brief Called when all loops are completed or execution is stopped.
  void finish()
  {
    running_ = false;
    goal_in_flight_ = false;
    RCLCPP_INFO(get_logger(), "Completed %zu loops.", repeat_count_);
  }

  // members
  int delay_ms_;
  std::string yaml_file_, frame_id_, action_name_;
  bool stop_on_fail_;

  bool running_, goal_in_flight_;
  bool paused_ = false;

  size_t repeat_count_, loop_idx_, wp_idx_;

  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  start_srv_, cancel_srv_, pause_srv_, resume_srv_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_;
};

/// \brief Main entry point for the WaypointLooper node.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointLooper>());
  rclcpp::shutdown();
  return 0;
}
