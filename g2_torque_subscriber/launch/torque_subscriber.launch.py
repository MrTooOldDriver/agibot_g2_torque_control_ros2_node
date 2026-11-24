"""Launch the G2 torque subscriber (C++ or Python implementation).

  ros2 launch g2_torque_subscriber torque_subscriber.launch.py urdf_path:=/path/g2.urdf
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py urdf_path:=...
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true urdf_path:=...

Arguments override the parameter file (config/torque_subscriber.yaml by default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    impl = LaunchConfiguration("impl").perform(context)
    if impl not in ("cpp", "py"):
        raise RuntimeError(f"impl must be 'cpp' or 'py', got '{impl}'")
    overrides = {}
    for name in ("urdf_path", "backend", "dry_run"):
        value = LaunchConfiguration(name).perform(context)
        if value != "":
            overrides[name] = value.lower() == "true" if name == "dry_run" else value
    return [Node(
        package="g2_torque_subscriber",
        executable="torque_subscriber_node" if impl == "cpp" else "torque_subscriber_node.py",
        name="g2_torque_subscriber",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        # Ctrl+C: the node sends its final frame and restores the previous control mode; give it
        # time before launch escalates to SIGTERM / SIGKILL.
        sigterm_timeout="10",
        sigkill_timeout="10",
        parameters=[LaunchConfiguration("params_file"), overrides],
    )]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory("g2_torque_subscriber"), "config",
                                  "torque_subscriber.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("impl", default_value="cpp", description="cpp or py"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("urdf_path", default_value="",
                              description="URDF for gravity compensation (overrides params_file)"),
        DeclareLaunchArgument("backend", default_value="",
                              description="C++ only: gdk | fake | <module path>"),
        DeclareLaunchArgument("dry_run", default_value="",
                              description="true: never switch mode or send torque"),
        OpaqueFunction(function=_node),
    ])
