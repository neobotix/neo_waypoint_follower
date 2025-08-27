
// ============================================================================
// WaypointLooper Control Semantics (ROS 2 style):
//
// - Stop: Cancels the current navigation goal and stops the loop immediately.
// - Pause: Temporarily halts waypoint execution, preserving current progress.
// - Resume: Continues execution from the paused waypoint and loop index.
// - Cancel: Aborts the loop and resets all progress; does not auto-restart.
// - Skip: Advances to the next waypoint, bypassing the current one.
// - Stop & Restart: Cancels all progress and immediately restarts the loop from the beginning.
// ============================================================================
// Neobotix GmbH
// Author: Adarsh Karan K P

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

    skip_srv_ = create_service<std_srvs::srv::Trigger>(
      "skip_current_waypoint",
      std::bind(&WaypointLooper::skipCallback, this, std::placeholders::_1, std::placeholders::_2));

    restart_srv_ = create_service<std_srvs::srv::Trigger>(
      "stop_restart_waypoint_loop",
      std::bind(&WaypointLooper::stopRestartCallback, this, std::placeholders::_1, std::placeholders::_2));

  }

private:
  // services
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
    restart_after_cancel_ = false;
    skip_after_cancel_ = false;
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

  void cancelCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (delay_timer_) delay_timer_->cancel();
    paused_ = false;
    running_ = false;
    goal_in_flight_ = false;
    restart_after_cancel_ = false;
    skip_after_cancel_ = false;

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

  void skipCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!running_) { res->success=false; res->message="Not running"; return; }
    if (paused_)   { res->success=false; res->message="Paused; resume first"; return; }

    if (goal_in_flight_ && current_goal_) {
      skip_after_cancel_ = true;
      auto fut = client_->async_cancel_goal(current_goal_); (void)fut;
      res->success = true; res->message = "Skipping current waypoint...";
      return;
    }

    // No goal in flight: just advance and send next
    ++wp_idx_;
    sendNext();
    res->success = true;
    res->message = "Skipped to next waypoint.";
  }

  void stopRestartCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (delay_timer_) delay_timer_->cancel();
    paused_ = false;
    restart_after_cancel_ = true;
    running_ = false;           // temporarily
    goal_in_flight_ = false;

    if (current_goal_) {
      auto fut = client_->async_cancel_goal(current_goal_); (void)fut;
      // result callback will see restart_after_cancel_ and kick fresh start
    } else {
      // immediate restart
      loop_idx_ = 0; wp_idx_ = 0;
      if (!loadYaml() || waypoints_.empty()) {
        restart_after_cancel_ = false;
        res->success=false; res->message="Failed to load YAML or no waypoints."; return;
      }
      if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
        restart_after_cancel_ = false;
        res->success=false; res->message="NavigateToPose server not up."; return;
      }
      running_ = true;
      sendNext();
      restart_after_cancel_ = false;
    }

    res->success = true;
    res->message = "Stop & restart requested.";
  }

  // yaml
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
          // clear the handle for safety
          if (restart_after_cancel_) {
            restart_after_cancel_ = false;
            loop_idx_ = 0; wp_idx_ = 0;

            if (!loadYaml() || waypoints_.empty()) { running_ = false; return; }
            if (!client_->wait_for_action_server(std::chrono::seconds(5))) { running_ = false; return; }

            running_ = true;
            sendNext();
            return;
          }
          if (skip_after_cancel_) {
            skip_after_cancel_ = false;
            ++wp_idx_; // move to next
            // fall through to send next (if still running)
          } else if (paused_) {
            // user pause: do nothing (stay paused at same wp_idx_)
            return;
          } else {
            // external cancel: treat as failure
            RCLCPP_WARN(this->get_logger(), "Goal canceled");
            if (stop_on_fail_) { running_ = false; finish(); return; }
            else { ++wp_idx_; }
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
  bool restart_after_cancel_ = false;
  bool skip_after_cancel_ = false;

  size_t repeat_count_, loop_idx_, wp_idx_;

  std::vector<std::pair<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
  start_srv_, cancel_srv_, pause_srv_, resume_srv_, skip_srv_, restart_srv_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointLooper>());
  rclcpp::shutdown();
  return 0;
}
