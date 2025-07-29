
# neo_waypoint_follower


![neo_waypoint_follower demo]([https://github.com/user-attachments/assets/aab52775-912b-4804-831a-cca1ad2c0df8](https://github.com/neobotix/neo_waypoint_follower/blob/main/media/waypoint_loop.gif))


## Overview

`neo_waypoint_follower` is a ROS 2 Humble package developed by Neobotix GmbH for waypoint following for Neobotix ROX robots.

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

---

### 2. `waypoint_looper`

- **Purpose:** Loads waypoints from a YAML file and sends navigation goals to the Nav2 stack, looping through the waypoints as configured.
- **Parameters:**
  - `yaml_file` (string): Path to the waypoints YAML file
  - `frame_id` (string): Frame ID for waypoints (default: `map`)
  - `repeat_count` (int): Number of times to repeat the waypoint loop (default: `100`)
  - `wait_at_waypoint_ms` (int): Time to wait at each waypoint in milliseconds (default: `500`)
  - `stop_on_failure` (bool): Stop looping on navigation failure (default: `true`)
  - `action_name` (string): Nav2 action server name (default: `/navigate_to_pose`)

- **Services:**
  - `/start_waypoint_loop` (`std_srvs/srv/Trigger`): Starts the waypoint loop
  - `/stop_waypoint_loop` (`std_srvs/srv/Trigger`): Stops the waypoint loop


## Launch

The package provides a launch file for easy configuration and startup:

```sh
ros2 launch neo_waypoint_follower waypoint_follower_launch.py
```

### Launch Arguments

| Argument            | Default Value                | Description                                         |
|---------------------|-----------------------------|-----------------------------------------------------|
| waypoints_topic     | `/waypoints`                | Topic to listen to for waypoints                    |
| waypoints_yaml      | `<pkg>/config/waypoints.yaml`       | Path to the waypoints YAML file (save & loop)       |
| frame_id            | `map`                       | Frame ID for waypoints                              |
| repeat_count        | `100`                       | Number of times to repeat the loop                  |
| wait_at_waypoint_ms | `500`                       | Time to wait at each waypoint (ms)                  |
| stop_on_failure     | `true`                      | Stop looping on navigation failure                  |            |

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
4. **Stop waypoint looping**:
   ```sh
   ros2 service call /stop_waypoint_loop std_srvs/srv/Trigger {}
   ```

## Further Information

For more details and usage examples, please visit our official documentation at: [https://neobotix-docs.de/](https://neobotix-docs.de/)

