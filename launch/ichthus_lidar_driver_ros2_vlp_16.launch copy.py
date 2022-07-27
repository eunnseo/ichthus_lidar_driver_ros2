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
  # frontend
  frontend_center_param_path = LaunchConfiguration("frontend_center_param_path").perform(context)
  with open(frontend_center_param_path, "r") as f:
    frontend_center_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_front_param_path = LaunchConfiguration("frontend_front_param_path").perform(context)
  with open(frontend_front_param_path, "r") as f:
    frontend_front_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_rear_param_path = LaunchConfiguration("frontend_rear_param_path").perform(context)
  with open(frontend_rear_param_path, "r") as f:
    frontend_rear_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_right_param_path = LaunchConfiguration("frontend_right_param_path").perform(context)
  with open(frontend_right_param_path, "r") as f:
    frontend_right_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  frontend_left_param_path = LaunchConfiguration("frontend_left_param_path").perform(context)
  with open(frontend_left_param_path, "r") as f:
    frontend_left_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  # backend
  backend_center_param_path = LaunchConfiguration("backend_center_param_path").perform(context)
  with open(backend_center_param_path, "r") as f:
    backend_center_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  backend_front_rear_param_path = LaunchConfiguration("backend_front_rear_param_path").perform(context)
  with open(backend_front_rear_param_path, "r") as f:
    backend_front_rear_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  backend_right_left_param_path = LaunchConfiguration("backend_right_left_param_path").perform(context)
  with open(backend_right_left_param_path, "r") as f:
    backend_right_left_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  # calibration
  calibration_2_param_path = LaunchConfiguration("calibration_2_param_path").perform(context)
  with open(calibration_2_param_path, "r") as f:
    calibration_2_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  calibration_3_param_path = LaunchConfiguration("calibration_3_param_path").perform(context)
  with open(calibration_3_param_path, "r") as f:
    calibration_3_param = yaml.safe_load(f)["/**"]["ros__parameters"]

  setup_type = LaunchConfiguration("setup_type").perform(context)
  composable_node_list = []
  if setup_type == "CENTER":
    print("setup type: CENTER")
    composable_node_list = [
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
        name='backend_center',
        parameters=[backend_center_param, 
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      )
    ]
  elif setup_type == "FRONT-REAR":
    print("setup type: FRONT-REAR")
    composable_node_list = [
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
        name='backend_front_rear',
        parameters=[backend_front_rear_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      )
    ]
  elif setup_type == "RIGHT-LEFT":
    print("setup type: RIGHT-LEFT")
    composable_node_list = [
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::frontend_node::FrontendNode',
        name='frontend_right',
        parameters=[frontend_right_param, calibration_2_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      ),
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::frontend_node::FrontendNode',
        name='frontend_left',
        parameters=[frontend_left_param, calibration_3_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      ),
      ComposableNode(
        package='ichthus_lidar_driver_ros2',
        plugin='ichthus_lidar_driver_ros2::backend_node::BackendNode',
        name='backend_right_left',
        parameters=[backend_right_left_param,
          {
            "use_sim_time": LaunchConfiguration("use_sim_time")
          }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
      )
    ]
  else:
    print("do nothing")

  container = ComposableNodeContainer(
    name='ichthus_lidar_driver_ros2_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=composable_node_list,
    output='screen',
  )

  group = GroupAction(
    [
      PushRosNamespace(""),
      container,
    ]
  )

  return [group]

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

  # parameter
  add_launch_arg("frontend_center_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_vlp_center.param.yaml",], "", )
  add_launch_arg("frontend_front_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_front.param.yaml",], "", )
  add_launch_arg("frontend_rear_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_rear.param.yaml",], "", )
  add_launch_arg("frontend_right_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_right.param.yaml",], "", )
  add_launch_arg("frontend_left_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/frontend_os1_left.param.yaml",], "", )

  add_launch_arg("backend_center_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/backend_vlp_center.param.yaml",], "", )
  add_launch_arg("backend_front_rear_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/backend_front_rear.param.yaml",], "", )
  add_launch_arg("backend_right_left_param_path", [FindPackageShare("ichthus_lidar_driver_ros2"), "/cfg/backend_right_left.param.yaml",], "", )
  add_launch_arg("setup_type", "CENTER") # FRONT-REAR, RIGHT-LEFT, CENTER

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
