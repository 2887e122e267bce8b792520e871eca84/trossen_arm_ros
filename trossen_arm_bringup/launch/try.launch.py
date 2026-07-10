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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,  # Moveit
    TimerAction,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

# Moveit Configs
from moveit_configs_utils import MoveItConfigsBuilder
import Jetson.GPIO as GPIO
import yaml
import os


GPIO.cleanup()


@dataclass
class ArmLaunchConfig:
    """Configuration for a single instance of a Trossen Arm in a multi-arm launch file."""

    robot_model: str
    robot_name: str
    arm_variant: Literal['base', 'leader', 'follower']
    arm_side: Literal['none', 'left', 'right']
    ip_address: str
    ros2_control_hardware_type: Literal['real', 'mock_components']
    ros2_controllers_config_parameter_filename: str
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    xyz: str
    rpy: str
    use_downdraft: bool
    use_suction_cup: bool


# --- Camera rig configuration --------------------------------------------
# Fill in real extrinsics (x, y, z, roll, pitch, yaw in the `world` frame)
# for each camera once measured/calibrated. Until then these are placeholders
# and the resulting point clouds will NOT be correctly positioned in world.
#
# stagger_delay is relative to CAMERA_BASE_DELAY (see below), not to launch
# start -- this keeps cameras from racing the arm controller bringup while
# still spacing out USB enumeration between the 4 devices.
CAMERAS = [
    {
        'name': 'cam_left',
        'serial_no': "'419122270123'",
        'tf_prefix': 'cam_left_',
        'stagger_delay': 0.0,
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    },
    {
        'name': 'cam_right',
        'serial_no': "'409122272701'",
        'tf_prefix': 'cam_right_',
        'stagger_delay': 3.0,
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    },
    {
        'name': 'cam_chute',
        'serial_no': "'412622272151'",
        'tf_prefix': 'cam_chute_',
        'stagger_delay': 6.0,
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    },
    {
        'name': 'cam_jogger',  # on the USB 2.0 hub -- gets the most headroom
        'serial_no': "'412622272127'",
        'tf_prefix': 'cam_jogger_',
        'stagger_delay': 10.0,
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    },
]

# Cameras start this many seconds after launch start, on top of their own
# stagger_delay, so the arms + controller_manager are already up first.
CAMERA_BASE_DELAY = 25.0

COMMON_ARGS = {
    'enable_color': 'true',
    'enable_depth': 'true',
    'depth_module.depth_profile': '640,480,30',
    'rgb_camera.color_profile': '640,480,30',
    'spatial_filter.enable': 'true',
    'temporal_filter.enable': 'true',
    'align_depth.enable': 'true',
    'publish_tf': 'true',
}


ROBOTS = [
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_1',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.4',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0, y=-0.25, z=0.0,
        roll=0.0, pitch=0.0, yaw=0.0,
        xyz="0.707 -0.1975 1.015",
        rpy="0 0 1.57",
        use_suction_cup=True,
        use_downdraft=True,
    ),
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_2',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.5',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0, y=-0.25, z=0.0,
        roll=0.0, pitch=0.0, yaw=0.0,
        xyz="0.443 -0.1975 1.015",
        rpy="0 0 1.57",
        use_suction_cup=False,
        use_downdraft=True,
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
                'arm_variant': robot.arm_variant,
                'arm_side': robot.arm_side,
                'ip_address': robot.ip_address,
                'ros2_control_hardware_type': robot.ros2_control_hardware_type,
                'xyz': robot.xyz,
                'rpy': robot.rpy,
                'use_suction_cup': 'true' if robot.use_suction_cup else 'false',
                'use_downdraft': 'true' if robot.use_downdraft else 'false',
                'use_world_frame': 'false',
            }
        )
        .robot_description_semantic(
            file_path='config/wxai.srdf.xacro',
            mappings={
                'prefix': f'{robot.robot_name}/',
                'variant': robot.arm_variant,
                'use_downdraft': 'true' if robot.use_downdraft else 'false',
                'use_suction_cup': 'true' if robot.use_suction_cup else 'false',
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
            pipelines=['ompl'],
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml',
        )
        .joint_limits(
            file_path=f'config/{robot.robot_name}_joint_limits.yaml',
        )
        .sensors_3d(
            file_path='config/sensors_3d.yaml',
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        namespace=robot.robot_name,
        executable='move_group',
        parameters=[moveit_configs.to_dict()],
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
        arguments=['-d', rviz_config_file_launch_arg],
        parameters=[moveit_configs.to_dict()],
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
            '--child-frame-id', f'/{robot.robot_name}/Bottom_Box' if robot.use_downdraft else f'/{robot.robot_name}/base_link',
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
        parameters=[ros2_control_controllers_config_parameter_file],
        remappings=[
            ('~/robot_description', f'/{robot.robot_name}/robot_description'),
        ],
        output={'both': 'screen'},
    )

    controller_spawner_nodes: list[Node] = []
    for controller_name in ['arm_controller', 'gripper_controller', 'joint_state_broadcaster']:
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
                        actions=[commander_server_node],
                    ),
                ],
            ),
        ),
    ]


def build_camera_actions() -> list[Action]:
    """Build staggered camera + world-TF actions for all entries in CAMERAS."""
    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )

    camera_actions: list[Action] = []

    for cam in CAMERAS:
        launch_args = dict(COMMON_ARGS)
        launch_args.update({
            'camera_name': cam['name'],
            'camera_namespace': cam['name'],
            'serial_no': cam['serial_no'],
            'tf_prefix': cam['tf_prefix'],
        })

        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments=launch_args.items(),
        )

        # Anchors this camera's frame tree into `world` -- without this the
        # camera's TF subtree is disconnected and its depth/point cloud data
        # can't be transformed into the world/robot frame.
        cam_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"{cam['name']}_static_transform_publisher",
            arguments=[
                '--x', str(cam['x']),
                '--y', str(cam['y']),
                '--z', str(cam['z']),
                '--roll', str(cam['roll']),
                '--pitch', str(cam['pitch']),
                '--yaw', str(cam['yaw']),
                '--frame-id', 'world',
                '--child-frame-id', f"{cam['tf_prefix']}link",
            ],
            output={'both': 'screen'},
        )

        camera_actions.append(
            TimerAction(
                period=CAMERA_BASE_DELAY + cam['stagger_delay'],
                actions=[include, cam_static_tf],
            )
        )

    return camera_actions


def launch_setup(context, *args, **kwargs):
    actions: list[Action] = []

    for i, robot in enumerate(ROBOTS):
        actions.extend(
            generate_launch_description_for_robot(context, robot, include_rviz=(i == 0))
        )

    actions.extend(build_camera_actions())

    shared_nodes = TimerAction(
        period=5.0,  # Give both arms time to finish spawning
        actions=[
            Node(
                package='main_controller',
                executable='main_controller',
                name='main_controller',
                output={'both': 'screen'},
            ),
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
            #     package='armor_record',
            #     executable='bag_recorder_node',
            #     name='bag_recorder_node',
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