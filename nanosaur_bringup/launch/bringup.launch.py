# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import Node


def load_config(config):
    if os.path.isfile(config):

        with open(config, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
    return {}


def generate_launch_description():
    pkg_bringup = get_package_share_directory('nanosaur_bringup')
    pkg_description = get_package_share_directory('nanosaur_description')
    pkg_control = get_package_share_directory('nanosaur_control')

    # Load nanosaur configuration and check if are included extra parameters
    # This feature is used to avoid pass configuration from docker
    conf = load_config(os.path.join(pkg_bringup, 'param', 'robot.yml'))

    # Load namespace from robot.yml
    namespace_conf = os.getenv("HOSTNAME") if conf.get("multirobot", False) else ""
    # Load cover_type from robot.yml
    cover_type_conf = conf.get("cover_type", 'fisheye')

    config_common_path = LaunchConfiguration('config_common_path')
    namespace = LaunchConfiguration('namespace')
    cover_type = LaunchConfiguration('cover_type')

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=os.path.join(pkg_bringup, 'param', 'nanosaur.yml'),
        description='Path to the `nanosaur.yml` file.')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value=namespace_conf,
        description='Enable a namespace for multiple robot. This namespace group all nodes and topics.')

    declare_cover_type_cmd = DeclareLaunchArgument(
        name='cover_type',
        default_value=cover_type_conf,
        description='Cover type to use. Options: pi, fisheye, realsense, zedmini.')

    jtop_node = Node(
        package='jetson_stats_wrapper',
        namespace=namespace,
        executable='jtop',
        name='jtop'
    )

    # System manager
    system_manager = Node(
        package='ros2_system_manager',
        namespace=namespace,
        executable='system_manager',
        name='system_manager'
    )

    nanosaur_base_node = Node(
        package='nanosaur_base',
        namespace=namespace,
        executable='nanosaur_base',
        name='nanosaur_base',
        respawn=True,
        respawn_delay=5,
        parameters=[config_common_path],
        output='screen'
    )

    nanosaur_camera_node = Node(
        package='nanosaur_camera',
        namespace=namespace,
        executable='nanosaur_camera',
        name='nanosaur_camera',
        respawn=True,
        respawn_delay=5,
        parameters=[config_common_path],
        output='screen'
    )

    # include another launch file in nanosaur namespace
    # https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html
    description_launch = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            # Nanosaur description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg_description, '/launch/description.launch.py']),
                launch_arguments={'cover_type': cover_type}.items()
            )
        ]
    )

    # include another launch file in nanosaur namespace
    twist_control_launch = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            # nanosaur twist launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_control, '/launch/twist_control.launch.py']))
        ]
    )

    joy2eyes_node = Node(
        package='nanosaur_base',
        namespace=namespace,
        executable='joy2eyes',
        name='joy2eyes',
        output='screen'
    )

    # https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/
    # include another launch file in the chatter_ns namespace
    teleop_launch = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg_control, '/launch/teleop.launch.py']),
                launch_arguments={'joy_vel': 'joy_vel',
                                  'config_filepath': os.path.join(pkg_control, 'param', 'ps3.nanosaur.yml')}.items()
            )
        ]
    )

    system_manager_node = Node(package='ros2_system_manager',
                               executable='joy2sm',
                               name='joy2sm',
                               parameters=[config_common_path],
                               output='screen')

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    # Namespace nanosaur
    ld.add_action(declare_namespace_cmd)
    # cover_type
    ld.add_action(declare_cover_type_cmd)
    # Nanosaur parameter yml file path
    ld.add_action(declare_config_common_path_cmd)
    # Nanosaur description launch
    ld.add_action(description_launch)
    # jtop node
    ld.add_action(jtop_node)
    # System manager
    ld.add_action(system_manager)
    # Nanusaur driver motors and display
    ld.add_action(nanosaur_base_node)

    # Nanosaur camera
    if conf.get("nanosaur_camera", False):
        ld.add_action(nanosaur_camera_node)

    # Twist control launcher
    if conf.get("no_twist_mux", False):
        print("Disable twist-mux")
    else:
        ld.add_action(twist_control_launch)

    # Extra Debug packages
    # - Eyes bridge
    if conf.get("debug", False):
        print("DEBUG variable exist - Load extra nodes")
        ld.add_action(joy2eyes_node)

    # teleoperation joystick nanosaur
    # only if joystick is connected
    if os.path.exists("/dev/input/js0"):
        print("Enable Joystick")
        # Teleoperation control
        ld.add_action(teleop_launch)
        # Run Joystick to system_manager node
        ld.add_action(system_manager_node)

    return ld
# EOF
