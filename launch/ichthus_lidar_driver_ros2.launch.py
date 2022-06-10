import os

import launch
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import OpaqueFunction

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

import ament_index_python

import yaml


def launch_setup(context, *args, **kwargs):
  frontend_front_param_path = LaunchConfiguration("frontend_front_param_path").perform(context)
  with open(frontend_front_param_path, "r") as f:
    frontend_front_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_rear_param_path = LaunchConfiguration("frontend_rear_param_path").perform(context)
  with open(frontend_rear_param_path, "r") as f:
    frontend_rear_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_center_param_path = LaunchConfiguration("frontend_center_param_path").perform(context)
  with open(frontend_center_param_path, "r") as f:
    frontend_center_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  backend_param_path = LaunchConfiguration("backend_param_path").perform(context)
  with open(backend_param_path, "r") as f:
    backend_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  calibration_2_param_path = LaunchConfiguration("calibration_2_param_path").perform(context)
  with open(calibration_2_param_path, "r") as f:
    calibration_2_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  calibration_3_param_path = LaunchConfiguration("calibration_3_param_path").perform(context)
  with open(calibration_3_param_path, "r") as f:
    calibration_3_param = yaml.safe_load(f)["/**"]["ros__parameters"]
  
  # can_odom_node = Node(
  #   package='odom_can',
  #   executable='odom_node',
  #   name='odom_can',
  #   parameters=[
  #     {
  #       "use_sim_time": LaunchConfiguration("use_sim_time")
  #     }],
  # )

  container = ComposableNodeContainer(
    name='ichthus_lidar_driver_ros2_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::frontend_node::FrontendNode',
        name='frontend_front',
        parameters=[frontend_front_param, calibration_2_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      ),
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::frontend_node::FrontendNode',
        name='frontend_rear',
        parameters=[frontend_rear_param, calibration_3_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      ),

      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::backend_node::BackendNode',
        name='backend',
        parameters=[backend_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      )
    ],
    output='screen',
  )

  center_container = ComposableNodeContainer(
    name='ichthus_lidar_driver_ros2_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::frontend_node::FrontendNode',
        name='frontend_center',
        parameters=[frontend_center_param, calibration_3_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      ),

      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::backend_node::BackendNode',
        name='backend',
        parameters=[backend_param, 
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      )
    ],
    output='screen',
  )

  group = GroupAction(
    [
      PushRosNamespace(""),
      # container,
      center_container,
    ]
  )

  return [group]
  # return [group, can_odom_node]

def get_param_file(package_name, file_name):
  """Pass the given param file as a LaunchConfiguration."""
  file_path = os.path.join(
    ament_index_python.get_package_share_directory(package_name),
    'cfg',
    file_name)
  return LaunchConfiguration(
    'params', default=[file_path])


def generate_launch_description():
  """Generate launch description with multiple components."""

  launch_arguments = []
  def add_launch_arg(name: str, default_value=None, description=None):
    launch_arguments.append(
      DeclareLaunchArgument(name, default_value=default_value, description=description)
    )

  add_launch_arg("frontend_front_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_front.param.yaml",], "", )
  add_launch_arg("frontend_rear_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_rear.param.yaml",], "", )
  add_launch_arg("frontend_center_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_center.param.yaml",], "", )
  add_launch_arg("backend_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/backend_default.param.yaml",], "", )
  add_launch_arg("calibration_2_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/OSI64_2.yaml",], "", )
  add_launch_arg("calibration_3_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/OSI64_3.yaml",], "", )

  # component
  add_launch_arg("use_intra_process", "true", "use ROS2 component container communication")
  add_launch_arg("use_multithread", "true", "use multithread")
  add_launch_arg("use_sim_time", "true")

  set_container_executable = SetLaunchConfiguration(
    "container_executable",
    "component_container",
    condition=UnlessCondition(LaunchConfiguration("use_multithread")),
  )
  set_container_mt_executable = SetLaunchConfiguration(
    "container_executable",
    "component_container_mt",
    condition=IfCondition(LaunchConfiguration("use_multithread")),
  )

  return launch.LaunchDescription(
    launch_arguments
    + [
      set_container_executable,
      set_container_mt_executable,
    ]
    + [OpaqueFunction(function=launch_setup)]
  )
