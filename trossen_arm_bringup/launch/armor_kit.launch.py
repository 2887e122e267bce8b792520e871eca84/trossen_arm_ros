# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from dataclasses import dataclass
from typing import Literal
from ament_index_python.packages import get_package_share_directory

from launch import Action, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction, # Moveit
    TimerAction,
    OpaqueFunction, 
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import (
    ParameterFile,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Moveit Configs
from moveit_configs_utils import MoveItConfigsBuilder
# import Jetson.GPIO as GPIO

# GPIO.cleanup()
ROBOT_TO_SIMULATE = 2  # 1 - left ; 2 - right


@dataclass
class ArmLaunchConfig:
    """Configuration for a single instance of a Trossen Arm in a multi-arm launch file."""

    robot_model: str
    """Robot model codename, such as `wxai`"""

    robot_name: str
    """Name of the robot, such as `trossen_arm_1`"""

    arm_variant: Literal['base', 'leader', 'follower']
    """End effector variant of the Trossen Arm, such as `base`, `leader`, or `follower`"""

    arm_side: Literal['none', 'left', 'right']
    """Side of the Trossen Arm, such as `none`, `left`, or `right`"""

    ip_address: str
    """IP address of the robot"""

    ros2_control_hardware_type: Literal['real', 'mock_components']
    """Type of ROS 2 control hardware interface, such as `real` or `mock_components`"""

    ros2_controllers_config_parameter_filename: str
    """Name of the ROS 2 controllers configuration file, such as `controllers.yaml`"""

    x: float
    """X coordinate of the robot base frame in meters measured in the world frame"""

    y: float
    """Y coordinate of the robot base frame in meters measured in the world frame"""

    z: float
    """Z coordinate of the robot base frame in meters measured in the world frame"""

    roll: float
    """Roll angle of the robot base frame in radians measured in the world frame"""

    pitch: float
    """Pitch angle of the robot base frame in radians measured in the world frame"""

    yaw: float
    """Yaw angle of the robot base frame in radians measured in the world frame"""



ROBOTS = [
    # Left
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_1',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.2',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0,
        y=0.0 if ROBOT_TO_SIMULATE == 1 else 0.345,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ),
    # Right
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_2',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.3',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0,
        y=0.0 if ROBOT_TO_SIMULATE == 2 else -0.345,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ),
]

def generate_launch_description_for_robot(
    context, robot: ArmLaunchConfig, include_rviz: bool = False
) -> list[Action]:
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')

    moveit_configs = (
        MoveItConfigsBuilder(
            robot_name='wxai',
            package_name='trossen_arm_moveit',
        )
        .robot_description(
            file_path=PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'urdf',
                'wxai.urdf.xacro',
            ]).perform(context),
            mappings={
                'prefix': f'{robot.robot_name}/',
                'variant': robot.arm_variant,
                'ip_address': robot.ip_address,
                'ros2_control_hardware_type': robot.ros2_control_hardware_type,
            }
        )
        .robot_description_semantic(
            file_path='config/wxai.srdf.xacro',
            mappings={
                'prefix': f'{robot.robot_name}/',
                'variant': robot.arm_variant,
            },
        )
        .planning_scene_monitor(
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_planning_scene=True,
        )
        .trajectory_execution(
            file_path=f'config/{robot.robot_name}_moveit_controllers.yaml',
            moveit_manage_controllers=True,
        )
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=[
                'ompl',
            ],
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml',
        )
        .joint_limits(
            file_path=f'config/{robot.robot_name}_joint_limits.yaml',
        )
        .sensors_3d(
            file_path=f'config/sensors_3d.yaml',
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        namespace=robot.robot_name,
        executable='move_group',
        parameters=[
            moveit_configs.to_dict(),
        ],
        remappings=[
            ('~/robot_description', f'/{robot.robot_name}/robot_description'),
        ],
        output={'both': 'screen'},
    )

    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2_dual',
        namespace=robot.robot_name,
        arguments=[
            '-d', rviz_config_file_launch_arg,
        ],
        parameters=[
            moveit_configs.to_dict(),
        ],
        output={'both': 'screen'},
    ) if include_rviz else None

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{robot.robot_name}_static_transform_publisher',
        arguments=[
            '--x', str(robot.x),
            '--y', str(robot.y),
            '--z', str(robot.z),
            '--roll', str(robot.roll),
            '--pitch', str(robot.pitch),
            '--yaw', str(robot.yaw),
            '--frame-id', 'world',
            '--child-frame-id', f'{robot.robot_name}/base_link',
        ],
        output={'both': 'screen'},
    )

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'config',
            robot.ros2_controllers_config_parameter_filename,
        ]),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot.robot_name,
        parameters=[
            ros2_control_controllers_config_parameter_file,
        ],
        remappings=[
            ('~/robot_description', f'/{robot.robot_name}/robot_description'),
        ],
        output={'both': 'screen'},
    )

    controller_spawner_nodes: list[Node] = []
    for controller_name in [
        'arm_controller',
        'gripper_controller',
        'joint_state_broadcaster',
    ]:
        controller_spawner_nodes.append(
            Node(
                name=f'{controller_name}_spawner',
                package='controller_manager',
                namespace=robot.robot_name,
                executable='spawner',
                arguments=[
                    controller_name,
                    '--controller-manager', f'/{robot.robot_name}/controller_manager',
                ],
                output={'both': 'screen'},
            )
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot.robot_name,
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output={'both': 'screen'},
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    commander_server_node = Node(
        package='armor_commander_cpp',
        executable='commander_server',
        name='commander_server',
        namespace=robot.robot_name,
        parameters=[
            moveit_configs.robot_description,  
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
        ],
        output={'both': 'screen'},
    )

    last_spawner = controller_spawner_nodes[-1]

    return [
        static_transform_node,
        controller_manager_node,
        robot_state_publisher_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=controller_spawner_nodes,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=last_spawner,
                on_exit=[
                    move_group_node,
                    *([moveit_rviz_node] if moveit_rviz_node is not None else []),
                    TimerAction(
                        period=3.0,
                        actions=[
                            commander_server_node,
                        ],
                    ),
                ],
            ),
        ),
    ]

def launch_setup(context, *args, **kwargs):
    actions = []
    for i, robot in enumerate(ROBOTS):
        robot_actions = generate_launch_description_for_robot(context, robot, include_rviz=(i == ROBOT_TO_SIMULATE-1))
        actions.extend(robot_actions)
    
    shared_nodes = TimerAction(
        period=5.0,  # Give both arms time to finish spawning
        actions=[
            # Node(
            #     package='main_controller',
            #     executable='main_controller',
            #     name='main_controller',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='ads48_bridge',
            #     name='ads48_bridge',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='ads49_bridge',
            #     name='ads49_bridge',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='monitor_ups',
            #     name='ups_monitor',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='pickup_action_server',
            #     name='pick_up',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='start_button',
            #     name='start_button',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='estop_button',
            #     name='estop_button',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='vibration_rack',
            #     name='vibration_rack',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_control_py',
            #     executable='pause_button',
            #     name='pause_button',
            #     output={'both': 'screen'},
            # ),
            # Node(
            #     package='armor_record',
            #     executable='recorder_node',
            #     name='recorder_node',
            #     output={'both': 'screen'},
            # ),
        ]
    )

    actions.append(shared_nodes)
    return actions


def generate_launch_description() -> LaunchDescription:
    use_rviz_launch_arg = DeclareLaunchArgument(
        'use_moveit_rviz',
        default_value='true',
        choices=('true', 'false'),
        description="Launches RViz with MoveIt's RViz configuration.",
    )

    rvizconfig_launch_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'rviz',
            'armor_kit.rviz',
        ]),
        description='Full path to the RVIZ config file to use.',
    )

    ld = LaunchDescription()
    ld.add_action(use_rviz_launch_arg)
    ld.add_action(rvizconfig_launch_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld