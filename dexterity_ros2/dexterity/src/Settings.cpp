/*
 * Settings.cpp
 */

#include <dexterity/Settings.hpp>

#include <type_traits>

namespace
{
// Declares the parameter with the current value as default. If it was already declared from a parameter
// file (automatically_declare_parameters_from_overrides), its value is read instead. Integers are
// accepted for double parameters, so "planning_timeout: 200" works as well as "200.0".
template <typename T>
void declareAndGet(rclcpp::Node& node, const std::string& name, T& value)
{
	if (!node.has_parameter(name))
	{
		value = node.declare_parameter<T>(name, value);
		return;
	}
	const rclcpp::Parameter parameter = node.get_parameter(name);
	if constexpr (std::is_same_v<T, double>)
	{
		if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
		{
			value = static_cast<double>(parameter.as_int());
			return;
		}
	}
	value = parameter.get_value<T>();
}
}

void DexteritySettings::declareAndLoad(rclcpp::Node& node)
{
	declareAndGet(node, "simulator_mode", simulatorMode);

	declareAndGet(node, "robot.move_group", robot.moveGroup);
	declareAndGet(node, "robot.home_state", robot.homeState);
	declareAndGet(node, "robot.trajectory_action", robot.trajectoryAction);
	declareAndGet(node, "robot.image_topic", robot.imageTopic);
	declareAndGet(node, "robot.camera_info_topic", robot.cameraInfoTopic);
	declareAndGet(node, "robot.base_frame", robot.baseFrame);
	declareAndGet(node, "robot.camera_optical_frame", robot.cameraOpticalFrame);

	declareAndGet(node, "perception.tracker_name", perception.trackerName);
	declareAndGet(node, "perception.tracker_mode", perception.trackerMode);
	declareAndGet(node, "perception.object_model_filename", perception.objectModelFilename);
	declareAndGet(node, "perception.min_accuracy_threshold", perception.minAccuracyThreshold);
	declareAndGet(node, "perception.object_side", perception.objectSide);
	declareAndGet(node, "perception.tag_size", perception.tagSize);
	declareAndGet(node, "perception.tag_family", perception.tagFamily);
	declareAndGet(node, "perception.pose_estimation_method", perception.poseEstimationMethod);
	declareAndGet(node, "perception.external_pose_topic", perception.externalPoseTopic);
	declareAndGet(node, "perception.show_tracking_window", perception.showTrackingWindow);

	declareAndGet(node, "controller.operation_type", controller.operationType);
	declareAndGet(node, "controller.open_loop_mode", controller.openLoopMode);
	declareAndGet(node, "controller.orientation_lock", controller.orientationLock);
	declareAndGet(node, "controller.inspection_distance", controller.inspectionDistance);
	declareAndGet(node, "controller.inspection_pose_error_margin", controller.inspectionPoseErrorMargin);
	declareAndGet(node, "controller.inspection_orientation_error_margin", controller.inspectionOrientationErrorMargin);
	declareAndGet(node, "controller.gripper_frame_id", controller.gripperFrameId);
	declareAndGet(node, "controller.truncation_type", controller.truncationType);
	declareAndGet(node, "controller.planning_timeout", controller.planningTimeout);
	declareAndGet(node, "controller.log_file", controller.logFile);
}

/*
 * Copies the values of a goal into the settings. Unlike ROS 1, empty strings and zero numbers are
 * treated as "not set" and keep the configured value, so a goal sent from the command line only needs
 * the fields that should change. Booleans cannot be "not set" and are always taken from the goal.
 */
void DexteritySettings::applyGoal(const dexterity_msgs::action::Dexterity::Goal& goal)
{
	auto setString = [](std::string& target, const std::string& value) { if (!value.empty()) target = value; };
	auto setNumber = [](double& target, double value) { if (value > 0) target = value; };

	if (goal.command != "update_parameters" && goal.command != "stop" && goal.command != "reset")
	{
		setString(controller.operationType, goal.command);
	}

	simulatorMode = goal.simulator_mode;
	controller.openLoopMode = goal.open_loop_mode;
	controller.orientationLock = goal.orientation_lock;

	setString(controller.truncationType, goal.truncation_type);
	setString(controller.gripperFrameId, goal.gripper_frame_id);
	setNumber(controller.planningTimeout, goal.planning_timeout);
	setNumber(controller.inspectionPoseErrorMargin, goal.inspection_pose_error_margin);
	setNumber(controller.inspectionOrientationErrorMargin, goal.inspection_orientation_error_margin);
	setNumber(controller.inspectionDistance, goal.inspection_distance);

	setString(perception.trackerName, goal.tracker_name);
	setString(perception.trackerMode, goal.tracker_mode);
	setString(perception.objectModelFilename, goal.object_model);
	setString(perception.objectSide, goal.object_side);
	setNumber(perception.minAccuracyThreshold, goal.accuracy);
	setNumber(perception.tagSize, goal.tag_size);
	setString(perception.tagFamily, goal.tag_family);
	setString(perception.poseEstimationMethod, goal.pose_estimation_method);
}
