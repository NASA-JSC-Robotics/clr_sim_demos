/**
 *  Copyright (c) 2025, United States Government, as represented by the
 *  Administrator of the National Aeronautics and Space Administration.
 *
 *  All rights reserved.
 *
 *  This software is licensed under the Apache License, Version 2.0
 *  (the "License"); you may not use this file except in compliance with the
 *  License. You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 */

#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

#include "dex_ivr_interfaces/srv/blob_centroid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::placeholders;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("demo_exec");

/* Struct that defines a single waypoint in the demonstration and information
 * about how to execute it. */
struct Waypoint {
  geometry_msgs::msg::Pose pose;
  std::vector<double> config;
  std::string planning_group;
  bool plan_cartesian;
  bool is_relative;
  bool is_preset;
  bool use_jconfig;
  std::string preset_name;
  std::string planner = "default";

  /* Constructor for waypoints with joint configuration information. */
  Waypoint(std::vector<double> j_config, std::string group, bool cartesian) {
    is_preset = false;
    use_jconfig = true;
    config = j_config;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = false;
  }

  /* Constructor for waypoints with geometry_msgs::msg::Pose. */
  Waypoint(geometry_msgs::msg::Pose wp_pose, std::string group, bool cartesian,
           bool relative = false) {
    is_preset = false;
    use_jconfig = false;
    pose = wp_pose;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = relative;
  }

  /* Constructor for waypoints with pose information. */
  Waypoint(float x, float y, float z, float qx, float qy, float qz, float qw,
           std::string group, bool cartesian, bool relative = false) {
    is_preset = false;
    use_jconfig = false;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = relative;
  }

  /* Constructor for preset waypoints, which do not require a pose to be set. */
  Waypoint(std::string name, std::string group, bool cartesian) {
    is_preset = true;
    use_jconfig = false;
    preset_name = name;
    planning_group = group;
    plan_cartesian = cartesian;
    is_relative = false;
  }
};

/*
 * The RunDemoNode executes a robot demonstration through a series of plan and
 * execute calls to MoveIt's MoveGroupInterface. Demo execution can be triggered
 * by calling the class function run_demo, which is a blocking call.
 */
class RunDemoNode : public rclcpp::Node {
public:
  RunDemoNode(rclcpp::NodeOptions node_options)
      : Node("demo_exec", node_options) {
    this->parameter_setup();
    color_blob_client =
        this->create_client<dex_ivr_interfaces::srv::BlobCentroid>(
            "color_blob_find");
    while (!color_blob_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(LOGGER, "Interrupted while waiting for the service.");
        return;
      }
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock(),
                                                  tf2::Duration(1000));
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

  void run_demo() {
    if (!this->init()) {
      RCLCPP_ERROR(LOGGER, "Failed to reach initial pose. Exiting.");
      return;
    }
    if (!this->stow()) {
      RCLCPP_ERROR(LOGGER, "Failed to stow manipulator. Exiting.");
      return;
    }
    if (!this->traverse()) {
      RCLCPP_ERROR(LOGGER, "Failed to traverse. Exiting.");
      return;
    }
    if (!this->approach_ctb()) {
      RCLCPP_ERROR(LOGGER, "Failed to reach CTB. Exiting.");
      return;
    }
    if (!this->approach_ctb_handle()) {
      RCLCPP_ERROR(LOGGER, "Failed to reach CTB handle. Exiting.");
      return;
    }
    if (!this->close_grasp()) {
      RCLCPP_ERROR(LOGGER, "Failed to grasp CTB handle. Exiting.");
      return;
    }
    if (!this->lift_ctb()) {
      RCLCPP_ERROR(LOGGER, "Failed to lift CTB. Exiting.");
      return;
    }
    RCLCPP_INFO(LOGGER, "Demo succeeded!");
  }

  bool init() {
    Waypoint approach_wp = Waypoint(-1.728, 0.135, 0.778, -0.714, 0.012, -0.042,
                                    0.699, "clr", false);
    approach_wp.planner = "RRTstarkConfigDefault";
    return plan_and_execute(approach_wp);
  }

  bool stow() {
    std::vector<double> approach_config = {-2.11185, -2.6529, 2.44346, 0.0, 1.0821, 3.26377};
    Waypoint approach_wp = Waypoint(approach_config, "ur_manipulator", false);
    approach_wp.planner = "RRTstarkConfigDefault";
    return plan_and_execute(approach_wp);
  }

  bool traverse() {
    std::vector<double> approach_config = {1.748};
    Waypoint approach_wp = Waypoint(approach_config, "rail", false);
    return plan_and_execute(approach_wp);
  }

  bool approach_ctb() {
    std::vector<double> approach_config = {1.748, 0.0, -1.88496, -2.35619, 2.21657, 0.0, 1.309, 3.22886};
    Waypoint approach_wp = Waypoint(approach_config, "clr", false);
    return plan_and_execute(approach_wp);
  }

  bool approach_ctb_handle() {
    // Find red blob in wrist camera image
    auto request =
        std::make_shared<dex_ivr_interfaces::srv::BlobCentroid::Request>();
    request->color = "red";
    auto future = color_blob_client->async_send_request(request);
    // Wait for the result.
    while (future.wait_for(std::chrono::milliseconds(100)) !=
           std::future_status::ready) {
      RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 1000,
                           "Waiting for color blob response...");
    }
    auto response = future.get();
    if (response->centroid_pose.header.frame_id != "") {
      RCLCPP_INFO(LOGGER, "Blob frame id: %s",
                  response->centroid_pose.header.frame_id.c_str());
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to find color blob.");
      return false;
    }

    // Transform waypoint from camera frame to planning frame
    geometry_msgs::msg::Pose local_pose = response->centroid_pose.pose;
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::Pose global_pose;
    bool success = this->get_global_transform(
        response->centroid_pose.header.frame_id, transform);
    if (!success) {
      return false;
    }
    tf2::doTransform(local_pose, global_pose, transform);

    // Add offset to place grasp frame below the CTB handle
    geometry_msgs::msg::Pose offset;
    offset.position.z = 0.04;
    global_pose = this->relative_to_global(global_pose, offset);

    Waypoint blob_wp = Waypoint(global_pose, "chonkur_grasp", true);
    return plan_and_execute(blob_wp);
  }

  bool close_grasp() {
    Waypoint close_grasp = Waypoint("close", "hand", true);
    return plan_and_execute(close_grasp);
  }

  bool lift_ctb() {
    Waypoint approach_wp = Waypoint(0.0, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0,
                                    "ur_manipulator", true, true);
    return plan_and_execute(approach_wp);
  }

  bool plan_and_execute(const Waypoint &waypoint) {
    move_group =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), waypoint.planning_group);

    if (waypoint.planner != "default") {
      move_group->setPlannerId(waypoint.planner);
    }

    moveit_viz = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), move_group->getPlanningFrame(),
        rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group->getRobotModel());

    this->mvt_setup();

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = false;
    int attempts = 0;

    while (!success && attempts < 3) {
      if (waypoint.is_preset || !waypoint.plan_cartesian) {
        success = this->plan_to_pose(waypoint, trajectory);
      } else {
        success = this->plan_cartesian(waypoint, trajectory);
      }
      attempts += 1;
    }

    if (!success) {
      return false;
    } else {
      if (wait_for_prompt) {
        this->prompt(trajectory);
      }
      success = this->execute_trajectory(trajectory);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return success;
  }

  bool wait_for_prompt;
  float scaling;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> moveit_viz;
  rclcpp::Client<dex_ivr_interfaces::srv::BlobCentroid>::SharedPtr
      color_blob_client;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

private:
  bool get_global_transform(const std::string &frame_id,
                            geometry_msgs::msg::TransformStamped &t) {
    try {
      t = this->tf_buffer->lookupTransform(move_group->getPlanningFrame(),
                                           frame_id, tf2::TimePointZero,
                                           std::chrono::nanoseconds(5000));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(),
                  "Could not get transform from %s to %s: %s", frame_id.c_str(),
                  move_group->getPlanningFrame().c_str(), ex.what());
      return false;
    }
    return true;
  }

  geometry_msgs::msg::Pose
  relative_to_global(const geometry_msgs::msg::Pose &start_pose,
                     const geometry_msgs::msg::Pose &end_pose) {
    auto pose_action_eigen =
        moveit_visual_tools::MoveItVisualTools::convertPose(end_pose);
    auto start_pose_eigen =
        moveit_visual_tools::MoveItVisualTools::convertPose(start_pose);
    auto end_pose_eigen = start_pose_eigen * pose_action_eigen;
    return moveit_visual_tools::MoveItVisualTools::convertPose(end_pose_eigen);
  }

  bool plan_cartesian(const Waypoint &waypoint,
                      moveit_msgs::msg::RobotTrajectory &trajectory) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s",
                move_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Using planning group: %s",
                move_group->getName().c_str());

    geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
    geometry_msgs::msg::Pose end_pose = waypoint.pose;

    if (waypoint.is_relative) {
      end_pose = this->relative_to_global(start_pose, end_pose);
    }

    std::vector<geometry_msgs::msg::Pose> waypoints = {end_pose};

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double trajectory_percent = move_group->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (trajectory_percent == 1.0) {
      RCLCPP_INFO(LOGGER, "Successfully computed 100%% of trajectory");
    } else {
      RCLCPP_ERROR(LOGGER,
                   "Error - Cartesian path plan (%.2f%% achieved). Cannot "
                   "execute trajectory.",
                   trajectory_percent * 100.0);
      return false;
    }

    robot_trajectory::RobotTrajectory rt(
        move_group->getCurrentState()->getRobotModel(),
        waypoint.planning_group);
    rt.setRobotTrajectoryMsg(*move_group->getCurrentState(), trajectory);

    // Cannot do velocity and acceleration scaling with Cartesian planning, as
    // described below:
    // https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html
    // The page below is referenced, which recommends manual velocity scaling:
    // https://groups.google.com/g/moveit-users/c/MOoFxy2exT4

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    bool success;
    success = iptp.computeTimeStamps(rt, scaling, scaling);
    RCLCPP_INFO(LOGGER, "Computed time stamp %s",
                success ? "SUCCEEDED" : "FAILED");

    if (success) {
      // Get RobotTrajectory_msg from RobotTrajectory
      rt.getRobotTrajectoryMsg(trajectory);
      RCLCPP_INFO(LOGGER, "Successfully scaled Cartesian trajectory");
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to scale Cartesian trajectory");
      return false;
    }
  }

  bool plan_to_pose(const Waypoint &waypoint,
                    moveit_msgs::msg::RobotTrajectory &trajectory) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    RCLCPP_INFO(LOGGER, "Planning frame: %s",
                move_group->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s",
                move_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Using planning group: %s",
                move_group->getName().c_str());

    if (waypoint.is_preset) {
      move_group->setJointValueTarget(
          move_group->getNamedTargetValues(waypoint.preset_name));
    } if (waypoint.use_jconfig) {
      move_group->setJointValueTarget(waypoint.config);
      move_group->setNumPlanningAttempts(10);
    } else {
      geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
      geometry_msgs::msg::Pose end_pose = waypoint.pose;
      if (waypoint.is_relative) {
        end_pose = this->relative_to_global(start_pose, end_pose);
      }
      move_group->setJointValueTarget(end_pose);
      move_group->setNumPlanningAttempts(10);
    }

    move_group->setMaxVelocityScalingFactor(scaling);
    move_group->setMaxAccelerationScalingFactor(scaling);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error_code = move_group->plan(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(LOGGER, "Successfully computed trajectory");
      trajectory = plan.trajectory_;
      return true;
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed with error code %s",
                   error_code_to_string(error_code).c_str());
      return false;
    }
  }

  bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    moveit::core::MoveItErrorCode move_success =
        move_group->execute(trajectory);
    if (move_success != moveit::core::MoveItErrorCode::SUCCESS) {
      error_code_to_string(move_success);
      return false;
    }
    return true;
  }

  void prompt(const moveit_msgs::msg::RobotTrajectory &trajectory) {
    // moveit_visual_tools fails to update state of rail and lift if they are
    // not in the current planning group.
    if (move_group->getName() == "clr") {
      moveit_viz->publishTrajectoryLine(
          trajectory, move_group->getRobotModel()->getJointModelGroup(
                          move_group->getName()));
    }
    moveit_viz->trigger();
    moveit_viz->prompt("Press next to perform motion");
  }

  void mvt_setup() {
    moveit_viz->loadMarkerPub();
    moveit_viz->enableBatchPublishing();
    moveit_viz->deleteAllMarkers();
    moveit_viz->loadRemoteControl();
  }

  void parameter_setup() {
    auto prompt_desc = rcl_interfaces::msg::ParameterDescriptor{};
    prompt_desc.description =
        "Whether to prompt before executing a trajectory.";
    auto scaling_desc = rcl_interfaces::msg::ParameterDescriptor{};
    prompt_desc.description =
        "Factor (<=1.0) by which to scale velocity and acceleration.";

    if (!this->has_parameter("wait_for_prompt")) {
      this->declare_parameter("wait_for_prompt", true, prompt_desc);
    }
    if (!this->has_parameter("scaling_factor")) {
      this->declare_parameter("scaling_factor", 1.0, scaling_desc);
    }
    wait_for_prompt = this->get_parameter("wait_for_prompt").as_bool();
    scaling = this->get_parameter("scaling_factor").as_double();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto run_demo_node = std::make_shared<RunDemoNode>(node_options);
  executor.add_node(run_demo_node);
  std::thread executor_thread(&rclcpp::executors::MultiThreadedExecutor::spin,
                              &executor);
  run_demo_node->run_demo();
  rclcpp::shutdown();
  executor_thread.join();
  return 0;
}
