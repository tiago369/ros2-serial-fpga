# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import launch
import launch_ros.actions
# import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    """Generate launch description with multiple components."""
    ros2_serial_fpga_directory = os.path.join(
        get_package_share_directory('ros2_serial_fpga'), 'config')

    param_config = os.path.join(ros2_serial_fpga_directory,
                                'test.yaml')

    bridge = launch_ros.actions.Node(
        name='ros2_to_serial_bridge',
        package='ros2_serial_example',
        executable='ros2_to_serial_bridge_node',
        parameters=[param_config],
        output='screen',
    )

    return launch.LaunchDescription([bridge])
