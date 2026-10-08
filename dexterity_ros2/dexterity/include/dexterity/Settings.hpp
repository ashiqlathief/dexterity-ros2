/*
 * Settings.hpp
 *
 * All dexterity settings in one place. In ROS 1 they lived on the global parameter server under
 * /Dexterity/... and every module read them from there. ROS 2 has no global parameter server, so the
 * node declares them as its own parameters, loads them into this struct, and hands the struct to the
 * perception and controller modules. An incoming action goal updates the struct (see applyGoal).
 */

#ifndef DEXTERITY_SETTINGS_HPP_
#define DEXTERITY_SETTINGS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <dexterity_msgs/action/dexterity.hpp>

// Robot specific names (MoveIt group, controller, camera, frames)
struct RobotSettings
{
	std::string moveGroup = "manipulator";
	std::string homeState = "Home";
	std::string trajectoryAction = "/joint_trajectory_controller/follow_joint_trajectory";
	std::string imageTopic = "/wrist_mounted_camera/image";
	std::string cameraInfoTopic = "/wrist_mounted_camera/camera_info";
	std::string baseFrame = "base_link";
	std::string cameraOpticalFrame = "color_optical_frame"; // Frame of the poses returned by the trackers (+Z forward)
};

struct PerceptionSettings
{
	std::string trackerName = "visp";               // visp | icg | m3t
	std::string trackerMode = "hybrid";             // me | klt | hybrid (ViSP only)
	std::string objectModelFilename = "cube_at_rl2.cao"; // In <dexterity share>/config/cad_models
	double minAccuracyThreshold = 80;               // Poses with a lower accuracy (in %) trigger a new track
	std::string objectSide = "origin";              // origin | top | front | right | left
	double tagSize = 0.072;                         // AprilTag size in m (black square)
	std::string tagFamily = "36h11";
	std::string poseEstimationMethod = "homography_virtual_vs";
	std::string externalPoseTopic = "/tracked_object_poses"; // icg / m3t pose estimation nodes publish here
	bool showTrackingWindow = true;                 // ViSP X11 window with the tracked object
};

struct ControllerSettings
{
	std::string operationType = "inspect_stationary"; // inspect_stationary | inspect_non_stationary | touch | grasp
	bool openLoopMode = false;
	bool orientationLock = false;
	double inspectionDistance = 0.3;           // m
	double inspectionPoseErrorMargin = 5;      // mm
	double inspectionOrientationErrorMargin = 15; // degrees
	std::string gripperFrameId = "dexterity_tool_frame"; // +X must point out of the gripper
	std::string truncationType = "half_trajectory"; // half_trajectory | quater_trajectory | step_by_step
	double planningTimeout = 200;              // ms
	std::string logFile = "controller_log.txt";
};

struct DexteritySettings
{
	bool simulatorMode = true;
	RobotSettings robot;
	PerceptionSettings perception;
	ControllerSettings controller;

	void declareAndLoad(rclcpp::Node& node);
	void applyGoal(const dexterity_msgs::action::Dexterity::Goal& goal);
};

#endif /* DEXTERITY_SETTINGS_HPP_ */
