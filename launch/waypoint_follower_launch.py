# Neobotix GmbH
# Author: Adarsh Karan K P 

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def execution_stage(context, 
                    waypoints_topic, 
                    save_waypoints_path, 
                    load_waypoints_path, 
                    frame_id, 
                    repeat_count, 
                    wait_at_waypoint_ms, 
                    stop_on_failure
                    ):

    waypoints_topic_val = str(waypoints_topic.perform(context))
    save_waypoints_path_val = str(save_waypoints_path.perform(context))
    load_waypoints_path_val = str(load_waypoints_path.perform(context))
    frame_id_val = frame_id.perform(context)
    repeat_count_val = int(repeat_count.perform(context))
    wait_at_waypoint_ms_val = int(wait_at_waypoint_ms.perform(context))
    stop_on_failure_val = bool(stop_on_failure.perform(context))

    save_waypoints_server_node = Node(
        package='neo_waypoint_follower',
        executable='save_waypoints_server',
        name='save_waypoints_server',
        output='screen',
        parameters=[{
            'waypoints_topic': waypoints_topic_val,
            'output_file': save_waypoints_path_val
        }]
    )

    waypoint_looper_node = Node(
        package='neo_waypoint_follower',
        executable='waypoint_looper',
        name='waypoint_looper',
        output='screen',
        parameters=[{
            'yaml_file': load_waypoints_path_val,
            'frame_id': frame_id_val,
            'repeat_count': repeat_count_val,
            'wait_at_waypoint_ms': wait_at_waypoint_ms_val,
            'stop_on_failure': stop_on_failure_val
        }]
    )

    vault_manager_node = Node(
        package='neo_waypoint_follower',
        executable='vault_manager',
        name='vault_manager',
        output='screen',
        parameters=[{
            'vault_dir': '/var/lib/neo/waypoints',
            'save_server_node': '/save_waypoints_server',
            'looper_node': '/waypoint_looper'
        }]
    )

    return [
        save_waypoints_server_node,
        waypoint_looper_node,
        vault_manager_node
    ]


def generate_launch_description():

    # Declare launch arguments
    declare_waypoints_topic = DeclareLaunchArgument(
        'waypoints_topic', default_value='/waypoints',
        description='Topic to listen to for waypoints'
    )

    declare_save_waypoints_path = DeclareLaunchArgument(
        'save_waypoints_path',
        default_value=os.path.join(
            get_package_share_directory('neo_waypoint_follower'),
            'config',
            'waypoints.yaml'
        ),
        description='Path to save waypoints YAML file'
    )

    declare_load_waypoints_path = DeclareLaunchArgument(
        'load_waypoints_path',
        default_value=os.path.join(
            get_package_share_directory('neo_waypoint_follower'),
            'config',
            'waypoints.yaml'
        ),
        description='Path to load waypoints YAML file for looping'
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='Frame ID for waypoints'
    )

    declare_repeat_count = DeclareLaunchArgument(
        'repeat_count', default_value='10',
        description='Number of times to repeat the loop'
    )

    declare_wait_at_waypoint_ms = DeclareLaunchArgument(
        'wait_at_waypoint_ms', default_value='500',
        description='Time to wait at each waypoint in milliseconds'
    )

    declare_stop_on_failure = DeclareLaunchArgument(
        'stop_on_failure', default_value='true',
        description='Stop looping on navigation failure'
    )

    opq_function = OpaqueFunction(function=execution_stage,
                                  args=[
                                      LaunchConfiguration('waypoints_topic'),
                                      LaunchConfiguration('save_waypoints_path'),
                                      LaunchConfiguration('load_waypoints_path'),
                                      LaunchConfiguration('frame_id'),
                                      LaunchConfiguration('repeat_count'),
                                      LaunchConfiguration('wait_at_waypoint_ms'),
                                      LaunchConfiguration('stop_on_failure')
                                  ])

    return LaunchDescription([
        declare_waypoints_topic,
        declare_save_waypoints_path,
        declare_load_waypoints_path,
        declare_frame_id,
        declare_repeat_count,
        declare_wait_at_waypoint_ms,
        declare_stop_on_failure,
        opq_function
    ])
