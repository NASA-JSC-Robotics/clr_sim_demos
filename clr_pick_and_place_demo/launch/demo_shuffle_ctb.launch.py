#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

from launch import LaunchDescription
from chonkur_deploy.launch_helpers import include_launch_file
from launch.actions import (
    DeclareLaunchArgument,
    Shutdown,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, NotSubstitution, Command, FindExecutable
from launch_ros.substitutions import (
    FindPackageShare,
)
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml
import tempfile
import random


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            try:
                return yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(exc)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        print("Was not able to load the yaml file at " + absolute_file_path)
        return None


def generate_launch_description():

    declared_arguments = []
    # this launch arg doesn't do anything right now because I don't want to enable passing
    # it through to the main control.launch.py... Same would go for something like headless
    # mode or for trying to increase running speed
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_pregenerated_mjcf",
            default_value="false",
            description="Use pre-generated mjcf instead of converting it on the fly.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="If the robot is running in simulation, use the published clock. /"
            "If this flag is false, the mockups managers will be launched.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "wait_for_prompt",
            default_value="true",
            description="Whether to prompt before executing a trajectory.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "scaling_factor",
            default_value="1.0",
            description="Factor (<=1.0) by which to scale velocity and acceleration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "waypoint_cfg",
            default_value="waypoints.yaml",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Optionally run headless, primarily for use in CI.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "randomize_ctb_orientation",
            default_value="false",
            description="Optionally randomize CTB orientation in sim",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctb_z_rotation",
            default_value="90",
            description="Optionally specify CTB orientation, ignored if randomize_ctb_orientation is true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "randomize_ctb_x",
            default_value="false",
            description="Optionally randomize the CTB's position along the length of CLR",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctb_x_position",
            default_value="0.8",
            description="Optionally specify CTB x position, along the length of the rail, \
                        specified in meters from world frame, recommend values in the range of (0.1 ,1.4), \
                        ignored if randomize_ctb_x is true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "randomize_ctb_y",
            default_value="false",
            description="Optionally randomize the CTB's distance from CLR",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctb_y_position",
            default_value="0.8",
            description="Optionally specify CTB y position, the distance between the CTB and the rail, \
                        specified in meters from world frame, recommend values in the range of (0.6, 0.95), \
                        ignored if randomize_ctb_x is true",
        )
    )

    sim = LaunchConfiguration("sim")
    wp_cfg_file_name = LaunchConfiguration("waypoint_cfg")
    waypoint_cfg = PathJoinSubstitution(
        [get_package_share_directory("clr_pick_and_place_demo"), "config", wp_cfg_file_name]
    )
    scaling_factor = LaunchConfiguration("scaling_factor")
    wait_for_prompt = LaunchConfiguration("wait_for_prompt")
    randomize_ctb_orientation = LaunchConfiguration("randomize_ctb_orientation")
    ctb_z_rotation = LaunchConfiguration("ctb_z_rotation")
    randomize_ctb_x = LaunchConfiguration("randomize_ctb_x")
    ctb_x_position = LaunchConfiguration("ctb_x_position")
    randomize_ctb_y = LaunchConfiguration("randomize_ctb_y")
    ctb_y_position = LaunchConfiguration("ctb_y_position")

    description_package = "clr_imetro_environments"
    description_file = "clr_trainer_multi_hatch.urdf.xacro"
    moveit_config_file_path = "srdf/clr_and_sim_mockups.srdf.xacro"
    description_full_path = os.path.join(get_package_share_directory(description_package), "urdf", description_file)

    # MUJOCO
    use_pregenerated_mjcf = LaunchConfiguration("use_pregenerated_mjcf")
    headless = LaunchConfiguration("headless")

    clr_mujoco_package_name = "clr_mujoco_config"
    clr_mujoco_description_file = "clr_mujoco_xacro.urdf"

    def generate_mjcf_description_and_node(context):

        ctb_orientation_randomized = randomize_ctb_orientation.perform(context)
        ctb_x_randomized = randomize_ctb_x.perform(context)
        ctb_y_randomized = randomize_ctb_y.perform(context)
        if ctb_orientation_randomized.lower() == "true":
            ctb_rot_z = random.uniform(0, 180)
            print(f"CTB Orientation is randomized \n\t CTB Orientation:{ctb_rot_z} (deg)")
        else:
            ctb_rot_z = ctb_z_rotation.perform(context)
        if ctb_x_randomized.lower() == "true":
            ctb_x = random.uniform(0.1, 1.4)
            print(f"CTB X Position is randomized \n\t CTB X Position:{ctb_x} (m)")
        else:
            ctb_x = ctb_x_position.perform(context)
        if ctb_y_randomized.lower() == "true":
            ctb_y = random.uniform(0.6, 0.95)
            print(f"CTB Y Position is randomized \n\t CTB Y Position:{ctb_y} (m)")
        else:
            ctb_y = ctb_y_position.perform(context)
        mjcf_robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "urdf", clr_mujoco_description_file]),
                # Grasp frames should not be converted to MJCF objects
                " add_grasp_push_frames:=false",
                " model_env:=true",
                " include_scene_objects:=true",
                " ctb_orientation_deg:=",
                str(ctb_rot_z),
                " ctb_position_x:=",
                str(ctb_x),
                " ctb_position_y:=",
                str(ctb_y),
            ]
        ).perform(context)

        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        tmp.write(mjcf_robot_description_content)
        tmp.close()

        # Ensure the file gets deleted
        def cleanup(event, context):
            if os.path.exists(tmp.name):
                os.remove(tmp.name)

        return [
            Node(
                package="mujoco_ros2_control",
                executable="make_mjcf_from_robot_description.py",
                output="both",
                emulate_tty=True,
                arguments=[
                    "--publish_topic",
                    "/mujoco_robot_description",
                    "--urdf",
                    tmp.name,
                    "--convert_stl_to_obj",
                    "--asset_dir",
                    PathJoinSubstitution([FindPackageShare(clr_mujoco_package_name), "description", "assets"]),
                ],
                condition=UnlessCondition(use_pregenerated_mjcf),
            ),
            RegisterEventHandler(OnShutdown(on_shutdown=cleanup)),
        ]

    generate_mjcf = OpaqueFunction(function=generate_mjcf_description_and_node)

    extra_xacro_args = [
        " use_pregenerated_mjcf:=",
        use_pregenerated_mjcf,
        " sim_speed:=",
        scaling_factor,
        " headless:=",
        headless,
    ]

    clr_launch = include_launch_file(
        package_name="clr_deploy",
        launch_file="control.launch.py",
        launch_arguments={
            "robot_description_package": "clr_mujoco_config",
            "robot_description_file": "clr_mujoco_xacro.urdf",
            "model_env": "true",
            "use_fake_hardware": "false",
            "use_sim_time": "true",
            "is_sim": "true",
            "control_node_package": "mujoco_ros2_control",
            "extra_xacro_args": extra_xacro_args,
        }.items(),
    )

    point_cloud_proc = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        remappings=[
            ("rgb/image_rect_color", "/wrist_mounted_camera/color/image_raw"),
            ("rgb/camera_info", "/wrist_mounted_camera/color/camera_info"),
            ("depth_registered/image_rect", "/wrist_mounted_camera/aligned_depth_to_color/image_raw"),
            ("points", "/wrist_mounted_camera/depth/color/points"),
        ],
    )

    moveit_config = (
        MoveItConfigsBuilder("clr", package_name="clr_moveit_config")
        .robot_description(file_path=description_full_path)
        .robot_description_semantic(file_path=moveit_config_file_path)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/clr_moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )

    package_share_dir = get_package_share_directory("clr_pick_and_place_demo")
    rviz_config_file = os.path.join(package_share_dir, "rviz", "demo_config.rviz")

    nodes_to_start = [
        Node(
            package="clr_pick_and_place_demo",
            executable="run_demo",
            output="both",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
                {"waypoint_cfg": waypoint_cfg},
                {"wait_for_prompt": wait_for_prompt},
                {"scaling_factor": scaling_factor},
                {"hw": NotSubstitution(sim)},
            ],
            on_exit=Shutdown(),
        ),
        Node(
            package="color_blob_centroid",
            executable="color_blob_node",
            output="both",
            parameters=[
                {
                    "mock_hardware": False,
                    "show_image": False,
                    "debug": False,
                    "continuous_output": False,
                    "use_sim_time": sim,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="both",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="both",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": sim},
            ],
        ),
    ]

    hw_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("clr_imetro_environments"), "launch", "mockups_managers.launch.py"
                )
            ),
            launch_arguments={
                "hatch_4040": "true",
                "trainer": "true",
                "second_trainer": "false",
                "tf_prefix": "",
            }.items(),
            condition=UnlessCondition(sim),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [generate_mjcf, clr_launch, point_cloud_proc] + nodes_to_start + hw_launch
    )
