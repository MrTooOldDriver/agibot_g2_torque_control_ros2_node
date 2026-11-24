"""Launch the G2 torque subscriber (C++ or Python implementation).

  ros2 launch g2_torque_subscriber torque_subscriber.launch.py
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py compensate_gravity:=true
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py impl:=py
  ros2 launch g2_torque_subscriber torque_subscriber.launch.py dry_run:=true

Arguments override the parameter file (config/torque_subscriber.yaml by default); an empty
argument keeps the file's value. The dynamics model defaults to the confirmed G2 URDF installed
with the package (share/g2_torque_subscriber/urdf/genie_g2_crs_omnipicker.urdf); pass
urdf_path:=... only to use another model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STRING_ARGS = ("urdf_path", "backend")
BOOL_ARGS = ("dry_run", "compensate_gravity", "compensate_coriolis", "compensate_inertia")


def _node(context):
    impl = LaunchConfiguration("impl").perform(context)
    if impl not in ("cpp", "py"):
        raise RuntimeError(f"impl must be 'cpp' or 'py', got '{impl}'")
    overrides = {}
    for name in STRING_ARGS + BOOL_ARGS:
        value = LaunchConfiguration(name).perform(context)
        if value == "":
            continue
        if name in BOOL_ARGS:
            if value.lower() not in ("true", "false"):
                raise RuntimeError(f"{name} must be true or false, got '{value}'")
            overrides[name] = value.lower() == "true"
        else:
            overrides[name] = value
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
                              description="URDF of the dynamics model (default: the package's "
                                          "urdf/genie_g2_crs_omnipicker.urdf)"),
        DeclareLaunchArgument("backend", default_value="",
                              description="gdk | fake (C++ also: <module path>). Python: "
                                          "must match the agibot_gdk on PYTHONPATH"),
        DeclareLaunchArgument("dry_run", default_value="",
                              description="true: never switch mode or send torque"),
        DeclareLaunchArgument("compensate_gravity", default_value="",
                              description="true: add g(q)"),
        DeclareLaunchArgument("compensate_coriolis", default_value="",
                              description="true: add C(q, dq) dq"),
        DeclareLaunchArgument("compensate_inertia", default_value="",
                              description="true: add M(q) ddq_d (~/<arm>/desired_accelerations)"),
        OpaqueFunction(function=_node),
    ])
