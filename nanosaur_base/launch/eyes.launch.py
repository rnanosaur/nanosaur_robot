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
import launch
from launch.substitutions import LaunchConfiguration
import launch_ros

def generate_launch_description():
    pkg_bringup = launch_ros.substitutions.FindPackageShare(package='nanosaur_bringup').find('nanosaur_bringup')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    
    nanosaur_config = os.path.join(pkg_bringup, 'param', 'nanosaur.yml')
    nanosaur_dir = LaunchConfiguration('nanosaur_dir', default=nanosaur_config)
    
    nanosaur_base_node = launch_ros.actions.Node(
        package='nanosaur_base',
        executable='nanosaur_base',
        name='nanosaur_base',
        parameters=[nanosaur_dir],
        output='screen'
    )

    joy2eyes_node = launch_ros.actions.Node(
        package='nanosaur_base',
        executable='joy2eyes',
        name='joy2eyes',
        output='screen'
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        # Joystick to eyes message wrapper
        joy2eyes_node,
        # Nanusaur driver motors and display
        nanosaur_base_node
    ])