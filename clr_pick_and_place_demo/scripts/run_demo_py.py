#!/usr/bin/env python3

# python libraries
import threading
import yaml

# generic ros libraries
import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
import tf_transformations as tf

from geometry_msgs.msg import Pose

from moveit.core.kinematic_constraints import construct_joint_constraint

from tf2_ros import (
    Buffer,
    TransformListener,
)


class Waypoint:
    def __init__(self):
        # Common fields
        self.pose = Pose()
        self.config = []
        self.planning_group = ""
        self.plan_cartesian = False
        self.is_relative = False
        self.is_preset = False
        self.use_jconfig = False
        self.preset_name = ""
        self.planner = "default"

    # ---- Joint configuration waypoint ----
    @classmethod
    def from_joint_config(cls, j_config, group: str, cartesian: bool):
        waypoint = cls()
        waypoint.is_preset = False
        waypoint.use_jconfig = True
        waypoint.config = list(j_config)
        waypoint.planning_group = group
        waypoint.plan_cartesian = cartesian
        waypoint.is_relative = False
        return waypoint

    # ---- Pose waypoint (Pose message) ----
    @classmethod
    def from_pose(cls, pose: Pose, group: str, cartesian: bool, relative: bool = False):
        waypoint = cls()
        waypoint.is_preset = False
        waypoint.use_jconfig = False
        waypoint.pose = pose
        waypoint.planning_group = group
        waypoint.plan_cartesian = cartesian
        waypoint.is_relative = relative
        return waypoint

    # ---- Pose waypoint (components) ----
    @classmethod
    def from_pose_components(
        cls,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        group: str,
        cartesian: bool,
        relative: bool = False,
    ):
        waypoint = cls()
        waypoint.is_preset = False
        waypoint.use_jconfig = False

        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw

        waypoint.planning_group = group
        waypoint.plan_cartesian = cartesian
        waypoint.is_relative = relative
        return waypoint

    # ---- Preset waypoint ----
    @classmethod
    def from_preset(cls, name: str, group: str):
        waypoint = cls()
        waypoint.is_preset = True
        waypoint.use_jconfig = False
        waypoint.preset_name = name
        waypoint.planning_group = group
        waypoint.plan_cartesian = False
        waypoint.is_relative = False
        return waypoint


def load_waypoints_from_yaml(demo_config: str):

    waypoint_map = dict()

    # Load YAML file
    with open(demo_config, "r") as f:
        waypoint_yaml = yaml.safe_load(f)

    if not waypoint_yaml or "waypoints" not in waypoint_yaml:
        get_logger("moveit_py").warn("Error loading YAML test/config")
        return

    for name, data in waypoint_yaml["waypoints"].items():

        # ---- Joint configuration waypoint ----
        if "config" in data:
            loaded_waypoint = Waypoint.from_joint_config(
                j_config=data["config"], group=data["planning_group"], cartesian=data["cartesian"]
            )

            if "planner" in data:
                loaded_waypoint.planner = data["planner"]

            waypoint_map[name] = loaded_waypoint

        # ---- Pose waypoint (x,y,z,qx,qy,qz,qw) ----
        elif "pose" in data:
            pose = data["pose"]

            loaded_waypoint = Waypoint.from_pose_components(
                x=pose["x"],
                y=pose["y"],
                z=pose["z"],
                qx=pose["qx"],
                qy=pose["qy"],
                qz=pose["qz"],
                qw=pose["qw"],
                group=data["planning_group"],
                cartesian=data["cartesian"],
                relative=data.get("relative", False),
            )

            if "planner" in data:
                loaded_waypoint.planner = data["planner"]

            waypoint_map[name] = loaded_waypoint

        # ---- Preset waypoint ----
        elif "preset_name" in data:
            loaded_waypoint = Waypoint.from_preset(name=data["preset_name"], group=data["planning_group"])

            if "planner" in data:
                loaded_waypoint.planner = data["planner"]

            waypoint_map[name] = loaded_waypoint

    return waypoint_map


def plan_to_configuration_name(waypoint, moveit_object):
    # get the planning component
    planning_component = moveit_object.get_planning_component(waypoint.planning_group)
    # set plan start state using predefined state
    planning_component.set_start_state_to_current_state()

    # set pose goal using predefined state
    planning_component.set_goal_state(configuration_name=waypoint.preset_name)

    plan_result = planning_component.plan()

    execute_plan(plan_result, moveit_object)


def plan_to_pose(waypoint, moveit_object, tf_buffer):
    # set plan start state to current state
    planning_component = moveit_object.get_planning_component(waypoint.planning_group)

    planning_component.set_start_state_to_current_state()
    jmg = moveit_object.get_robot_model().get_joint_model_group(waypoint.planning_group)
    tip_link = jmg.eef_name
    # get_logger("moveit_py").info(f"-----------------available planners: {planning_component.get_available_planners()}")
    plan_request_parameters = PlanRequestParameters(moveit_object, waypoint.planning_group)
    # plan_request_parameters.planner_id = "RRTConnectkConfigDefault"
    # plan_request_parameters.planning_pipeline = "ompl"
    plan_request_parameters.planner_id = "LIN"
    plan_request_parameters.planning_pipeline = "pilz_industrial_motion_planner"
    # planning_component.set_planner_id("PilzCartesianPlanner")

    # set pose goal with PoseStamped message
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "world"

    if waypoint.is_relative:
        # Note, technically this should work, but just returns an identity matrix.
        # Maybe bc robot state is not initialized properly?
        # ee_pose = robot_state.get_global_link_transform("base_link")

        # Get ee pose in world reference
        ee_pose_tf = tf_buffer.lookup_transform("world", tip_link, rclpy.time.Time())

        T_ee_pose = tf.quaternion_matrix(
            [
                ee_pose_tf.transform.rotation.x,
                ee_pose_tf.transform.rotation.y,
                ee_pose_tf.transform.rotation.z,
                ee_pose_tf.transform.rotation.w,
            ]
        )
        T_ee_pose[:3, 3] = [
            ee_pose_tf.transform.translation.x,
            ee_pose_tf.transform.translation.y,
            ee_pose_tf.transform.translation.z,
        ]

        T_waypoint = tf.quaternion_matrix(
            [
                waypoint.pose.orientation.x,
                waypoint.pose.orientation.y,
                waypoint.pose.orientation.z,
                waypoint.pose.orientation.w,
            ]
        )
        T_waypoint[:3, 3] = [
            waypoint.pose.position.x,
            waypoint.pose.position.y,
            waypoint.pose.position.z,
        ]

        pose_goal_np = T_ee_pose @ T_waypoint
        get_logger("moveit_py").info(f"-----------------goal: {pose_goal_np}")
        pose_goal_quat_tf = tf.quaternion_from_matrix(pose_goal_np)

        pose_goal.pose.position.x = pose_goal_np[0, 3]
        pose_goal.pose.position.y = pose_goal_np[1, 3]
        pose_goal.pose.position.z = pose_goal_np[2, 3]
        pose_goal.pose.orientation.x = pose_goal_quat_tf[0]
        pose_goal.pose.orientation.y = pose_goal_quat_tf[1]
        pose_goal.pose.orientation.z = pose_goal_quat_tf[2]
        pose_goal.pose.orientation.w = pose_goal_quat_tf[3]
    else:
        pose_goal.pose = waypoint.pose

    planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=tip_link)
    plan_result = planning_component.plan(single_plan_parameters=plan_request_parameters)

    execute_plan(plan_result, moveit_object)


def plan_to_joint_states(waypoint, moveit_object):
    # set plan start state to current state
    planning_component = moveit_object.get_planning_component(waypoint.planning_group)

    robot_model = moveit_object.get_robot_model()
    robot_state = RobotState(robot_model)
    jmg = robot_model.get_joint_model_group(waypoint.planning_group)
    joint_names = jmg.active_joint_model_names

    joint_values = dict()
    for i in range(0, len(joint_names)):
        joint_values[joint_names[i]] = waypoint.config[i]

    robot_state.joint_positions = joint_values

    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=robot_model.get_joint_model_group(waypoint.planning_group),
    )
    planning_component.set_start_state_to_current_state()
    planning_component.set_goal_state(motion_plan_constraints=[joint_constraint])

    plan_result = planning_component.plan()

    execute_plan(plan_result, moveit_object)


def plan_to_waypoint(waypoint, moveit_object, tf_buffer):
    if waypoint.is_preset:
        plan_to_configuration_name(waypoint, moveit_object)
    if waypoint.use_jconfig:
        plan_to_joint_states(waypoint, moveit_object)
    else:
        plan_to_pose(waypoint, moveit_object, tf_buffer)


def execute_plan(plan_result, moveit_object):
    # execute the plan
    if plan_result:
        get_logger("moveit_py").info("Executing plan")
        robot_trajectory = plan_result.trajectory
        moveit_object.execute(robot_trajectory, controllers=[])
    else:
        get_logger("moveit_py").error("Planning failed")


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    from rclpy.node import Node

    node = Node("test_node")
    executor = MultiThreadedExecutor()  # choose any number you want
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.declare_parameter("waypoint_cfg", "")
    waypoint_cfg = node.get_parameter("waypoint_cfg").value

    waypoint_map = load_waypoints_from_yaml(waypoint_cfg)

    logger = get_logger("moveit_py")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # instantiate MoveItPy instance and get planning component
    moveit_object = MoveItPy(node_name="moveit_py")
    logger.info("MoveItPy instance created")

    # moveit_object.getPlanningPipelines()
    plan_to_waypoint(waypoint_map["back_out"], moveit_object, tf_buffer)
    return
    plan_to_waypoint(waypoint_map["init"], moveit_object, tf_buffer)
    plan_to_waypoint(waypoint_map["approach_bench_seat"], moveit_object, tf_buffer)
    plan_to_waypoint(waypoint_map["pre_drop_ctb"], moveit_object, tf_buffer)


if __name__ == "__main__":
    main()
