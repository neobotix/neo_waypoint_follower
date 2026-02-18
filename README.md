# neo_waypoint_follower

![neo_waypoint_follower](https://github.com/user-attachments/assets/02c27bda-f1bc-4ca3-ad10-00a70c7917a8)

## Overview

`neo_waypoint_follower` is a ROS 2 package developed by Neobotix GmbH for waypoint following for Neobotix ROX robots.

It provides two core functionalities:

- **Waypoint Saving:** Save waypoints from RViz MarkerArray topics to a YAML file via a ROS 2 service.
- **Waypoint Looping:** Autonomously loop through saved waypoints using the Nav2 navigation stack, with flexible control over repetition, timing, and failure handling, via a ROS 2 service.


## Nodes

### 1. `save_waypoints_server`

- **Purpose:** Listens to a MarkerArray topic (e.g., from RViz) and saves the received waypoints to a YAML file on request.
- **Parameters:**
  - `waypoints_topic` (string): Topic to subscribe for waypoints (default: `/waypoints`)
  - `output_file` (string): Path to save the waypoints YAML file

- **Service:**
  - `/save_waypoints` (`std_srvs/srv/Trigger`): Saves the latest received waypoints to the specified YAML file.

- **Runtime note:**
  - The `output_file` parameter is read right before saving. You can change it at runtime:
    ```sh
    ros2 param set /save_waypoints_server output_file /home/user/my_waypoints.yaml
    ros2 service call /save_waypoints std_srvs/srv/Trigger {}
    ```

---

### 2. `waypoint_looper`

- **Purpose:** Loads waypoints from a YAML file and sends navigation goals to the Nav2 stack, looping through the waypoints as configured.
- **Modes:**
  - **Waypoint Loop Mode:** Loops through all waypoints for the specified number of repeats (`repeat_count`).
  - **Single Goal Mode:** If only one waypoint is loaded, the node switches to single-goal navigation and executes just that goal once.
- **Parameters:**
  - `yaml_file` (string): Path to the waypoints YAML file
  - `frame_id` (string): Frame ID for waypoints (default: `map`)
  - `repeat_count` (int): Number of times to repeat the waypoint loop (default: `100`)
  - `wait_at_waypoint_ms` (int): Time to wait at each waypoint in milliseconds (default: `200`)
  - `stop_on_failure` (bool): Stop looping on navigation failure (default: `true`)
  - `action_name` (string): Nav2 action server name (default: `/navigate_to_pose`)
  - `odom_topic` (string): Odometry topic for distance tracking (default: `/odom`)

- **Services:**
  - `/start_waypoint_loop` (`std_srvs/srv/Trigger`): Starts the waypoint loop or single-goal mode
  - `/pause_waypoint_loop` (`std_srvs/srv/Trigger`): Pauses the waypoint loop
  - `/resume_waypoint_loop` (`std_srvs/srv/Trigger`): Resumes the waypoint loop
  - `/cancel_waypoint_loop` (`std_srvs/srv/Trigger`): Cancels and resets the waypoint loop
  - `/publish_loaded_waypoints` (`std_srvs/srv/Trigger`): Publishes the currently loaded waypoints once to a latched topic

- **Topics:**
  - `/waypoint_loop/metrics` (`neo_waypoint_follower/msg/LooperMetrics`): Aggregated looper metrics (QoS: best_effort, durability_volatile)
  - `/waypoint_loop/loaded_waypoints` (`neo_waypoint_follower/msg/Waypoints`): Names + poses for the currently loaded waypoints (QoS: reliable, transient_local)

- **Message: `neo_waypoint_follower/LooperMetrics`**
  - `std_msgs/Header header`
  - Progress:
    - `uint32 loop_idx` (0-based current loop index)
    - `uint32 wp_idx` (0-based current waypoint index)
    - `string current_waypoint_name`
  - Distances (meters):
    - `float64 distance_from_start`
    - `float64 distance_from_last`
    - `float64 distance_remaining`
  - Timing:
    - `builtin_interfaces/Duration eta`
    - `builtin_interfaces/Duration navigation_time`
  - State:
    - `uint8 looper_state` — enum values:
      - `LOOPER_IDLE = 0`
      - `LOOPER_RUNNING = 1`
      - `LOOPER_WAITING = 2`
      - `LOOPER_PAUSED = 3`
      - `LOOPER_FINISHED = 4`
      - `LOOPER_ERROR = 5`
    - `uint8 nav_result_code` — raw `rclcpp_action::ResultCode` from latest NavigateToPose result
    - `uint16 nav_error_code` — `NavigateToPose.Result.error_code`
    - `string nav_error_msg` — `NavigateToPose.Result.error_msg`
    - `string status_message` — human-readable state/error summary

- **Message: `neo_waypoint_follower/Waypoints`**
  - `std_msgs/Header header`
  - `string[] names`
  - `geometry_msgs/PoseStamped[] poses`

- **Behavior Notes:**
  - Lowering `repeat_count` below the current loop index will cause the run to finish right after the current goal completes.
  - Changing `wait_at_waypoint_ms` does not affect an already running timer; it takes effect from the next waypoint.
  - If only one waypoint is loaded, single-goal mode is activated automatically.
  - Start-time busy conditions are reported by `/start_waypoint_loop` Trigger response (`success=false`, message).
  - Runtime action failures (for example ABORTED) are surfaced via `LOOPER_ERROR` + `nav_result_code/nav_error_code/nav_error_msg`.

- **Runtime note:**
  - The `yaml_file` parameter is read when you press Start. To switch waypoint files at runtime, set the param and call Start again:
    ```sh
    ros2 param set /waypoint_looper yaml_file /home/user/waypoints_alt.yaml
    ros2 service call /start_waypoint_loop std_srvs/srv/Trigger {}
    ```


## Launch

The package provides a launch file for easy configuration and startup:

```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py
```

### Launch Arguments

| Argument            | Default Value                  | Description                                         |
|---------------------|--------------------------------|-----------------------------------------------------|
| waypoints_topic     | `/waypoints`                   | Topic to listen to for waypoints                    |
| save_waypoints_path | `<pkg>/config/waypoints.yaml`  | Path used by `save_waypoints_server`                |
| load_waypoints_path | `<pkg>/config/waypoints.yaml`  | Path used by `waypoint_looper`                      |
| frame_id            | `map`                          | Frame ID for waypoints                              |
| repeat_count        | `100`                          | Number of times to repeat the loop                  |
| wait_at_waypoint_ms | `200`                          | Time to wait at each waypoint (ms)                  |
| stop_on_failure     | `true`                         | Stop looping on navigation failure                  |

Example:
```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py repeat_count:=10 wait_at_waypoint_ms:=1000
```

---


## Usage

### 1. Launch the Simulation and Navigation Stack

Open separate terminals and run the following commands in order:

**a. Launch the simulation:**
```sh
ros2 launch rox_bringup bringup_sim_launch.py
```

**b. Launch the navigation stack (with simulation time):**
```sh
ros2 launch rox_navigation navigation.launch.py use_sim_time:=True
```

**c. Launch RViz:**
```sh
ros2 launch neo_nav2_bringup rviz_launch.py
```

**d. Launch the waypoint follower (with example arguments):**
```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py repeat_count:=10 wait_at_waypoint_ms:=1000
```

You can also load a custom waypoints YAML file by specifying the `load_waypoints_path` launch argument. For example:
```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py load_waypoints_path:=/home/adarsh/waypoints_test.yaml
```
Make sure your custom YAML file follows the template provided in `config/waypoints_template.yaml`.

---

### 2. Add and Use Waypoints

1. **Add waypoints in RViz using the Navigation2 Panel.**  
   First select the `Waypoint / Nav Thru Poses Mode`. Then, when you add waypoints using the `Nav2 Goal` markers, the waypoints are automatically published to the `/waypoints` topic by the Nav2 stack.  
   Once you have finished marking all desired waypoints in RViz, *do not* choose any options from the Nav2 panel. Proceed to the next step.
2. **Save waypoints** by calling the service:
   ```sh
   ros2 service call /save_waypoints std_srvs/srv/Trigger {}
   ```
3. **Start waypoint looping**:
   ```sh
   ros2 service call /start_waypoint_loop std_srvs/srv/Trigger {}
   ```
   More Controls:

  - **Pause then resume:**
    ```sh
    ros2 service call /pause_waypoint_loop std_srvs/srv/Trigger {}
    ros2 service call /resume_waypoint_loop std_srvs/srv/Trigger {}
    ```

  - **Cancel & reset (no auto-restart):**
    ```sh
    ros2 service call /cancel_waypoint_loop std_srvs/srv/Trigger {}
    ```
5. **Publish loaded waypoints to a latched topic**

   Trigger the node to publish the currently loaded waypoints once, then echo them:

   ```sh
   ros2 service call /publish_loaded_waypoints std_srvs/srv/Trigger {}
   ros2 topic echo /waypoint_loop/loaded_waypoints
   ```
4. **Dynamic Parameter Update**

You can now change parameters at runtime using:

  ```sh
  ros2 param set /waypoint_looper repeat_count 2
  ros2 param set /waypoint_looper wait_at_waypoint_ms 0
  ```

  This allows you to adjust the loop count and wait time while the node is running. See "Behavior Notes" above for details on how these changes affect execution.


## Further Information

For more details and usage examples, please visit our official documentation at: [https://neobotix-docs.de/](https://neobotix-docs.de/)

---

## Vault Manager

Minimal, production-ready waypoint vault utilities bundled with `neo_waypoint_follower`.

### Purpose

- Owns a simple on-disk vault directory for waypoint YAMLs.
- Provides services to list, save, load into the looper (params only), and preview waypoints to a latched topic.
- Saves route-level metadata (`name`, `description`) together with waypoints in one YAML write.
- Preserves route sequence by writing waypoints in Marker ID order (avoids `wp_1, wp_10, wp_2` lexicographic jumps).

### Defaults

- Vault directory: `/var/lib/neo/waypoints`
- Preview topic: `/vault/preview_waypoints` (QoS: reliable, transient_local)
- Works alongside existing nodes: `save_waypoints_server` and `waypoint_looper`.

### Node: `vault_manager`

- Parameters:
  - `vault_dir` (string, default: `/var/lib/neo/waypoints`): Base directory that stores waypoint YAML files.
  - `looper_node` (string, default: `/waypoint_looper`): Target node name for setting `yaml_file` (and optional loop params).
  - `frame_id` (string, default: `map`): Frame used when previewing waypoints.
  - `waypoints_topic` (string, default: `/waypoints`): MarkerArray topic cached for `/vault/save_current`.

- Services:
  - `/vault/list` (`neo_waypoint_follower/srv/VaultList`)
    - Request: empty
    - Response: `bool success`, `string message`, `string[] filenames`, `int32[] points`, `bool[] has_loop`, `string[] names`, `string[] descriptions`
  - `/vault/save_current` (`neo_waypoint_follower/srv/VaultSaveCurrent`)
    - Request: `string filename`, `bool allow_overwrite`, `string name`, `string description`
    - Response: `bool success`, `string message`
    - Behavior: reads latest MarkerArray from `waypoints_topic`, writes `<vault_dir>/<filename>.yaml` with `metadata` + `waypoints` in one pass
  - `/vault/load_to_looper` (`neo_waypoint_follower/srv/VaultLoadToLooper`)
    - Request: `string filename`, `bool set_loop_params`, `int32 loop_count`, `int32 wait_ms`
    - Response: `bool success`, `string message`
    - Behavior: sets `/waypoint_looper` parameter `yaml_file` and (optionally) `repeat_count`, `wait_at_waypoint_ms`
  - `/vault/preview_once` (`neo_waypoint_follower/srv/VaultPreview`)
    - Request: `string filename`
    - Response: `bool success`, `string message`
    - Behavior: publishes a one-shot `neo_waypoint_follower/Waypoints` to `/vault/preview_waypoints` (latched)

- Topics:
  - `/vault/preview_waypoints` (`neo_waypoint_follower/msg/Waypoints`): One-shot preview published with QoS `transient_local` so late subscribers receive it.

### YAML Structure

`waypoints` is required. `metadata` is optional

```yaml
metadata:
  name: Warehouse Patrol
  description: Nightly perimeter route
waypoints:
  point_1:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  point_2:
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
```

### Usage Examples

- List vault files:
  ```sh
  ros2 service call /vault/list neo_waypoint_follower/srv/VaultList {}
  ```

- Save current waypoints with metadata:
  ```sh
  ros2 service call /vault/save_current neo_waypoint_follower/srv/VaultSaveCurrent \
    "{filename: patrol_a, allow_overwrite: true, name: 'Warehouse Patrol', description: 'Night route'}"
  ```

- Load into looper (set params only):
  ```sh
  ros2 service call /vault/load_to_looper neo_waypoint_follower/srv/VaultLoadToLooper \
    "{filename: patrol_a, set_loop_params: true, loop_count: 3, wait_ms: 500}"
  ```

- Preview once (latched):
  ```sh
  ros2 service call /vault/preview_once neo_waypoint_follower/srv/VaultPreview "{filename: patrol_a}"
  ros2 topic echo /vault/preview_waypoints
  ```

### Launch Integration

`vault_manager` is included in `waypoint_follower_launch.py` with sensible defaults. Override its parameters if needed:

```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py \
  vault_dir:=/var/lib/neo/waypoints
```
