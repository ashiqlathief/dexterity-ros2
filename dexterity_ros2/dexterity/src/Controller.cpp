/*
 * Controller.cpp
 */

#include <dexterity/Controller.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <dexterity/HelpFunctions.hpp>

namespace
{
const std::string POIFrameId = "object_frame";
}

Controller::Controller(rclcpp::Node::SharedPtr node, std::shared_ptr<MoveGroupInterface> moveGroup,
                       std::shared_ptr<tf2_ros::Buffer> tfBuffer, rclcpp::CallbackGroup::SharedPtr callbackGroup)
	: node(node), moveGroupInterface(moveGroup), tfBuffer(tfBuffer), tfBroadcaster(node)
{
	trajectoryActionName = node->get_parameter("robot.trajectory_action").as_string();
	trajectoryActionClient = rclcpp_action::create_client<FollowJointTrajectory>(node, trajectoryActionName, callbackGroup);
	eventPublisher = node->create_publisher<dexterity_msgs::msg::DexterityEvent>("~/events", 50);
}

void Controller::publishEvent(dexterity_msgs::msg::DexterityEvent event)
{
	event.stamp = node->now();
	eventPublisher->publish(event);
}

void Controller::setParameters(const DexteritySettings& settings)
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	const auto& s = settings.controller;

	// Fresh state for a new run
	state = WAITING_FOR_POSE;
	finalSequence = false;
	interimPoseReached = false;
	numberOfPlanningSuccesses = 0;
	awaitingFreshPose = false;
	++trajectoryGoalSequence;

	logFilename = s.logFile;
	writeToLog("*****************************************");
	writeToLog("New Run: " + getCurrentDateTimeString());

	openLoopMode = s.openLoopMode;
	writeToLog(std::string("Open Loop Mode = ") + (openLoopMode ? "true" : "false"));
	orientationLock = s.orientationLock;
	moveGroupInterface->setPlanningTime(s.planningTimeout / 1000.0);
	inspectionDistance = s.inspectionDistance;
	writeToLog("Inspection Distance = " + std::to_string(inspectionDistance));
	gripperFrameId = s.gripperFrameId;
	baseFrame = settings.robot.baseFrame;
	inspectionPoseErrorMargin = s.inspectionPoseErrorMargin / 1000.0;
	inspectionOrientationErrorMargin = dtor(s.inspectionOrientationErrorMargin);

	if (s.operationType == "inspect_stationary")
	{
		operationType = INSPECT_STATIONARY;
	}
	else if (s.operationType == "inspect_non_stationary")
	{
		operationType = INSPECT_NON_STATIONARY;
	}
	else if (s.operationType == "touch")
	{
		operationType = TOUCH;
	}
	else if (s.operationType == "grasp")
	{
		// The ROS 1 node never had a grasp final sequence either, and nothing here commands the gripper
		throw std::runtime_error("Operation type 'grasp' is not implemented (available: inspect_stationary, inspect_non_stationary, touch)");
	}
	else if (s.operationType == "stop")
	{
		operationType = STOP;
	}
	else
	{
		throw std::runtime_error("Unknown operation type '" + s.operationType + "'");
	}
	writeToLog("Operation Type = " + getOperationTypeString());

	if (s.truncationType == "step_by_step")
	{
		truncationType = STEP_BY_STEP;
	}
	else if (s.truncationType == "half_trajectory")
	{
		truncationType = HALF_TRAJECTORY;
	}
	else if (s.truncationType == "quater_trajectory" || s.truncationType == "quarter_trajectory")
	{
		truncationType = QUARTER_TRAJECTORY;
	}
	else
	{
		throw std::runtime_error("Unknown truncation type '" + s.truncationType + "'");
	}

	RCLCPP_INFO(node->get_logger(), "Controller: %s, inspection distance %.3f m, margins %.1f mm / %.1f deg, gripper frame %s, truncation %s, open loop %s",
		getOperationTypeString().c_str(), inspectionDistance, s.inspectionPoseErrorMargin, s.inspectionOrientationErrorMargin,
		gripperFrameId.c_str(), getTruncationTypeString().c_str(), openLoopMode ? "on" : "off");
}

void Controller::setCurrentObjectPose(const ObjectPose& objectPose)
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	if (objectPose.poseValidity)
	{
		// Publish the object frames and also put them straight into the buffer, so they can be used right away
		tfBroadcaster.sendTransform({objectPose.coordinateFrame, objectPose.POIFrame});
		tfBuffer->setTransform(objectPose.coordinateFrame, "dexterity", false);
		tfBuffer->setTransform(objectPose.POIFrame, "dexterity", false);

		// After a move the pose of the image taken before it is out of date: the object frame hangs off the
		// camera, so the old pose would make the gripper-object distance look unchanged. Wait for a newer image.
		if (awaitingFreshPose)
		{
			if (rclcpp::Time(objectPose.coordinateFrame.header.stamp, node->get_clock()->get_clock_type()) <= freshPoseAfter)
			{
				return;
			}
			awaitingFreshPose = false;
		}

		if (readyForFinalSequence())
		{
			state = FINAL_PLANNING;
			performFinalSequence();
			return;
		}

		state = INTERIM_PLANNING;
		if (!finalSequence)
		{
			if (!calculateInterimTargetPose(objectPose.coordinateFrame))
			{
				state = WAITING_FOR_POSE;
				return;
			}
			planTrajectory();
			if (state == INTERIM_EXECUTION)
			{
				if (!openLoopMode)
				{
					truncateTrajectory();
				}
				sendGoal();
			}
		}
	}
	else if (supervisor.isEnabled)
	{
		RCLCPP_INFO(node->get_logger(), "No valid object pose, searching for the object");
		state = SEARCHING_FOR_OBJECT;
		searchTargetPose = supervisor.lookUpForObject(moveGroupInterface->getCurrentPose());
		planTrajectory();
		if (state == SEARCHING_FOR_OBJECT)
		{
			sendGoal();
		}
	}
}

/*
 * Calculates the end-effector pose that takes the gripper from its current pose to the interim
 * inspection pose near the object.
 */
bool Controller::calculateInterimTargetPose(const geometry_msgs::msg::TransformStamped&)
{
	bool tfOk = true;
	// Inspection pose: on the Z axis of the object frame, at the inspection distance
	geometry_msgs::msg::PoseStamped objectInspectionPose;
	objectInspectionPose.header.frame_id = POIFrameId;
	objectInspectionPose.pose.position.z = inspectionDistance;
	objectInspectionPose.pose.orientation.w = 1;

	// Current end-effector pose (MoveIt) expressed in the gripper frame (identity unless they differ)
	geometry_msgs::msg::PoseStamped currentGripperPose = moveGroupInterface->getCurrentPose();
	geometry_msgs::msg::PoseStamped currentGripperPoseWRTGripperFrame = changeBaseFrame(gripperFrameId, currentGripperPose, *tfBuffer, &tfOk);

	// Inspection pose expressed in the gripper frame
	geometry_msgs::msg::PoseStamped objectInspectionPoseWRTGripperFrame = changeBaseFrame(gripperFrameId, objectInspectionPose, *tfBuffer, &tfOk);

	// Translation from the gripper frame to the inspection pose
	geometry_msgs::msg::PoseStamped targetPoseWRTGripper;
	targetPoseWRTGripper.header.frame_id = gripperFrameId;
	targetPoseWRTGripper.pose.position = objectInspectionPoseWRTGripperFrame.pose.position;

	// Rotation from the gripper to the inspection pose
	if (orientationLock)
	{
		targetPoseWRTGripper.pose.orientation = objectInspectionPoseWRTGripperFrame.pose.orientation;
	}
	else
	{
		tf2::Quaternion qShortestRotation = findShortestRotation(&tfOk).normalize();
		targetPoseWRTGripper.pose.orientation = tf2::toMsg(qShortestRotation);
	}

	// Transform that moves the gripper from the origin of its own frame to the target pose
	geometry_msgs::msg::TransformStamped additionalTransform;
	additionalTransform.header.frame_id = gripperFrameId;
	additionalTransform.child_frame_id = "target_orientation";
	additionalTransform.header.stamp = node->now();
	additionalTransform.transform.translation.x = targetPoseWRTGripper.pose.position.x;
	additionalTransform.transform.translation.y = targetPoseWRTGripper.pose.position.y;
	additionalTransform.transform.translation.z = targetPoseWRTGripper.pose.position.z;
	additionalTransform.transform.rotation = targetPoseWRTGripper.pose.orientation;

	if (!tfOk)
	{
		RCLCPP_WARN(node->get_logger(), "Missing transforms (%s, %s), not planning", gripperFrameId.c_str(), POIFrameId.c_str());
		return false;
	}
	tf2::doTransform(currentGripperPoseWRTGripperFrame, interimTargetPoseWRTGripper, additionalTransform);
	return true;
}

bool Controller::calculateFinalTargetPose(double perpendicularDistance)
{
	bool tfOk = true;
	geometry_msgs::msg::PoseStamped currentGripperPose = moveGroupInterface->getCurrentPose();
	geometry_msgs::msg::PoseStamped currentGripperPoseWRTGripperFrame = changeBaseFrame(gripperFrameId, currentGripperPose, *tfBuffer, &tfOk);

	// Move along the gripper's +X axis (pointing into the scene) by the perpendicular distance
	geometry_msgs::msg::TransformStamped additionalTransform;
	additionalTransform.header.frame_id = gripperFrameId;
	additionalTransform.child_frame_id = "target_orientation";
	additionalTransform.header.stamp = node->now();
	additionalTransform.transform.translation.x = perpendicularDistance;
	additionalTransform.transform.rotation.w = 1;

	if (!tfOk)
	{
		RCLCPP_WARN(node->get_logger(), "Missing transform for %s, not planning", gripperFrameId.c_str());
		return false;
	}
	tf2::doTransform(currentGripperPoseWRTGripperFrame, finalTargetPoseWRTGripper, additionalTransform);
	return true;
}

/*
 * Gripper direction (+X of the gripper frame) and approach direction (-Z of the object frame),
 * both expressed in the gripper frame.
 */
void Controller::computeGripperAndObjectVectors(geometry_msgs::msg::Vector3Stamped& gripperVector, geometry_msgs::msg::Vector3Stamped& objectVector, bool* tfOk)
{
	geometry_msgs::msg::PointStamped gripperStart, gripperEnd, objectStart, objectEnd;
	gripperStart.header.frame_id = gripperFrameId;
	gripperEnd.header.frame_id = gripperFrameId;
	gripperEnd.point.x = 1;
	objectStart.header.frame_id = POIFrameId;
	objectStart.point.z = 1;
	objectEnd.header.frame_id = POIFrameId;

	auto gripperStartG = changeBaseFramePoint(gripperFrameId, gripperStart, *tfBuffer, tfOk);
	auto gripperEndG = changeBaseFramePoint(gripperFrameId, gripperEnd, *tfBuffer, tfOk);
	auto objectStartG = changeBaseFramePoint(gripperFrameId, objectStart, *tfBuffer, tfOk);
	auto objectEndG = changeBaseFramePoint(gripperFrameId, objectEnd, *tfBuffer, tfOk);

	gripperVector.header.frame_id = gripperFrameId;
	gripperVector.vector.x = gripperEndG.point.x - gripperStartG.point.x;
	gripperVector.vector.y = gripperEndG.point.y - gripperStartG.point.y;
	gripperVector.vector.z = gripperEndG.point.z - gripperStartG.point.z;

	objectVector.header.frame_id = gripperFrameId;
	objectVector.vector.x = objectEndG.point.x - objectStartG.point.x;
	objectVector.vector.y = objectEndG.point.y - objectStartG.point.y;
	objectVector.vector.z = objectEndG.point.z - objectStartG.point.z;
}

tf2::Quaternion Controller::findShortestRotation(bool* tfOk)
{
	geometry_msgs::msg::Vector3Stamped gripperVec, objectVec;
	computeGripperAndObjectVectors(gripperVec, objectVec, tfOk);
	updateVectorMarkers(objectVec, gripperVec);

	tf2::Vector3 gripper(gripperVec.vector.x, gripperVec.vector.y, gripperVec.vector.z);
	tf2::Vector3 object(objectVec.vector.x, objectVec.vector.y, objectVec.vector.z);
	gripper.normalize();
	object.normalize();

	// Quaternion of the shortest rotation from the gripper vector to the object vector
	tf2::Vector3 axisOfRotation = gripper.cross(object);
	if (axisOfRotation.length2() < 1e-12 && gripper.dot(object) < 0)
	{
		// Pointing exactly away from the object: any axis perpendicular to the gripper vector gives a half turn
		axisOfRotation = gripper.cross(std::abs(gripper.z()) < 0.9 ? tf2::Vector3(0, 0, 1) : tf2::Vector3(0, 1, 0)).normalized();
		return tf2::Quaternion(axisOfRotation, M_PI);
	}
	double w = std::sqrt(gripper.length2() * object.length2()) + gripper.dot(object);
	return tf2::Quaternion(axisOfRotation.x(), axisOfRotation.y(), axisOfRotation.z(), w);
}

// Damped least squares IK started at the joints of the state, so it can only converge to the solution next to them.
// (The KDL solver of ur_moveit_config also returns solutions with the shoulder or wrist flipped.)
bool Controller::solveIKLocally(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const Eigen::Isometry3d& target)
{
	const auto* link = state.getLinkModel(moveGroupInterface->getEndEffectorLink());
	std::vector<double> q;
	state.copyJointGroupPositions(group, q);
	const int n = static_cast<int>(q.size());
	const double damping = 0.05;
	const double maxStep = 0.2; // rad per iteration

	for (int iteration = 0; iteration < 500; ++iteration)
	{
		Eigen::Isometry3d current = state.getGlobalLinkTransform(link);
		Eigen::Matrix<double, 6, 1> error;
		error.head<3>() = target.translation() - current.translation();
		Eigen::AngleAxisd rotationError(target.rotation() * current.rotation().transpose());
		error.tail<3>() = rotationError.axis() * rotationError.angle();
		if (error.head<3>().norm() < 1e-4 && error.tail<3>().norm() < 1e-3)
		{
			return true;
		}

		Eigen::MatrixXd jacobian;
		state.getJacobian(group, link, Eigen::Vector3d::Zero(), jacobian);
		Eigen::MatrixXd jjt = jacobian * jacobian.transpose() + damping * damping * Eigen::MatrixXd::Identity(6, 6);
		Eigen::VectorXd step = jacobian.transpose() * jjt.ldlt().solve(error);
		double largest = step.cwiseAbs().maxCoeff();
		if (largest > maxStep)
		{
			step *= maxStep / largest;
		}
		for (int i = 0; i < n; ++i)
		{
			q[i] += step[i];
		}
		state.setJointGroupPositions(group, q);
		state.enforceBounds(group);
		state.update();
		state.copyJointGroupPositions(group, q);
	}
	return false;
}

bool Controller::setJointGoalNearCurrent(const geometry_msgs::msg::PoseStamped& poseWRTBase)
{
	auto state = moveGroupInterface->getCurrentState(2.0);
	if (!state)
	{
		return false;
	}
	const auto* group = state->getJointModelGroup(moveGroupInterface->getName());
	std::vector<double> current;
	state->copyJointGroupPositions(group, current);

	// The pose is in baseFrame, the robot state works in the root frame of the model
	Eigen::Isometry3d poseInBase;
	tf2::fromMsg(poseWRTBase.pose, poseInBase);
	Eigen::Isometry3d target = state->getGlobalLinkTransform(poseWRTBase.header.frame_id) * poseInBase;

	if (!solveIKLocally(*state, group, target))
	{
		return false;
	}

	std::vector<double> goal;
	state->copyJointGroupPositions(group, goal);
	const auto& joints = group->getActiveJointModels();
	double largestChange = 0;
	for (size_t i = 0; i < goal.size() && i < current.size() && i < joints.size(); ++i)
	{
		// Same angle, as close as possible to the current one (the UR joints turn +-2 pi)
		double value = goal[i];
		const auto& bounds = joints[i]->getVariableBounds()[0];
		while (value - current[i] > M_PI && value - 2 * M_PI >= bounds.min_position_)
		{
			value -= 2 * M_PI;
		}
		while (value - current[i] < -M_PI && value + 2 * M_PI <= bounds.max_position_)
		{
			value += 2 * M_PI;
		}
		goal[i] = value;
		largestChange = std::max(largestChange, std::abs(value - current[i]));
	}
	RCLCPP_INFO(node->get_logger(), "IK goal found, largest joint change %.2f rad", largestChange);
	if (largestChange > maxJointChange)
	{
		RCLCPP_WARN(node->get_logger(), "The IK solution needs a joint to turn by %.2f rad, the target is on the other side of the arm. Not moving", largestChange);
		return false;
	}
	moveGroupInterface->setStartStateToCurrentState();
	moveGroupInterface->setJointValueTarget(goal);
	return true;
}

void Controller::planTrajectory()
{
	geometry_msgs::msg::PoseStamped targetPose;
	if (state == FINAL_PLANNING)
	{
		targetPose = finalTargetPoseWRTGripper;
	}
	else if (state == INTERIM_PLANNING)
	{
		targetPose = interimTargetPoseWRTGripper;
	}
	else if (state == SEARCHING_FOR_OBJECT)
	{
		targetPose = searchTargetPose;
	}

	bool tfOk = true;
	geometry_msgs::msg::PoseStamped targetPoseWRTBase = findPoseWRTFrame(baseFrame, targetPose, *tfBuffer, &tfOk);
	if (!tfOk)
	{
		RCLCPP_WARN(node->get_logger(), "No transform from %s to %s, not planning", targetPose.header.frame_id.c_str(), baseFrame.c_str());
		abandonPlanning();
		return;
	}
	updateTargetPoseMarker(targetPoseWRTBase);
	moveGroupInterface->setStartStateToCurrentState();
	// A pose goal is solved with random IK seeds and can end up on the other side of the arm (shoulder turned by 180
	// degrees, wrist flipped, joints a full turn away), which swings the camera away from the object. Solve the IK
	// seeded with the current joints, take the equivalent angles closest to them, and plan to that joint goal.
	if (!setJointGoalNearCurrent(targetPoseWRTBase))
	{
		RCLCPP_WARN(node->get_logger(), "No IK solution near the current joints, not planning");
		abandonPlanning();
		return;
	}

	MoveGroupInterface::Plan fullPlan;
	auto start = std::chrono::steady_clock::now();
	bool success = static_cast<bool>(moveGroupInterface->plan(fullPlan));
	auto end = std::chrono::steady_clock::now();

	dexterity_msgs::msg::DexterityEvent event;
	event.event = dexterity_msgs::msg::DexterityEvent::PLANNING;
	event.controller_state = state == FINAL_PLANNING ? "final" : state == INTERIM_PLANNING ? "interim" : "search";
	event.success = success;
	event.planning_time = std::chrono::duration<double>(end - start).count();
	event.distance_to_object = estimateDistanceToObject();
	publishEvent(event);
	if (success)
	{
		++numberOfPlanningSuccesses;
		writeToLog("Planning Success " + std::to_string(numberOfPlanningSuccesses) + ": Distance to Target Pose: " + std::to_string(estimateDistanceToObject()));
		if (state == INTERIM_PLANNING)
		{
			state = INTERIM_EXECUTION;
		}
		else if (state == FINAL_PLANNING)
		{
			state = FINAL_EXECUTION;
		}
		plan = fullPlan;
	}
	else
	{
		RCLCPP_WARN(node->get_logger(), "Planning failed after %ld ms",
			std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
		abandonPlanning();
	}
}

// Back to waiting for a new object pose, from where the whole cycle starts again
void Controller::abandonPlanning()
{
	if (state == INTERIM_PLANNING || state == SEARCHING_FOR_OBJECT || state == FINAL_PLANNING)
	{
		state = WAITING_FOR_POSE;
	}
}

void Controller::truncateTrajectory()
{
	auto& points = plan.trajectory.joint_trajectory.points;
	if (truncationType == STEP_BY_STEP)
	{
		if (points.size() > 3)
		{
			points.resize(3);
		}
	}
	else if (truncationType == HALF_TRAJECTORY)
	{
		if (points.size() >= 4)
		{
			points.erase(points.begin() + points.size() / 2, points.end());
		}
	}
	else if (truncationType == QUARTER_TRAJECTORY)
	{
		if (points.size() >= 8)
		{
			points.erase(points.begin() + points.size() / 4, points.end());
		}
	}

	// The truncated trajectory ends mid-motion. The ROS 2 joint_trajectory_controller rejects trajectories
	// that do not end at rest, so the arm stops at the last point.
	if (!points.empty())
	{
		auto& last = points.back();
		std::fill(last.velocities.begin(), last.velocities.end(), 0.0);
		std::fill(last.accelerations.begin(), last.accelerations.end(), 0.0);
	}
}

void Controller::sendGoal()
{
	if (state != INTERIM_EXECUTION && state != FINAL_EXECUTION && state != SEARCHING_FOR_OBJECT)
	{
		throw std::runtime_error("Controller tried to send a trajectory while not executing. State: " + getStateString());
	}
	if (!trajectoryActionClient->action_server_is_ready())
	{
		RCLCPP_ERROR(node->get_logger(), "Trajectory action server %s is not available", trajectoryActionName.c_str());
		state = WAITING_FOR_POSE;
		return;
	}

	FollowJointTrajectory::Goal goal;
	goal.trajectory = plan.trajectory.joint_trajectory;
	unsigned int sequence = ++trajectoryGoalSequence;

	rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
	options.goal_response_callback = [this, sequence](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr& handle)
	{
		if (!handle)
		{
			RCLCPP_ERROR(node->get_logger(), "Trajectory goal was rejected");
			goalCompletionCallback(sequence, false);
		}
	};
	options.result_callback = [this, sequence](const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult& result)
	{
		bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
		if (!succeeded)
		{
			RCLCPP_WARN(node->get_logger(), "Trajectory did not succeed (result code %d): %s", static_cast<int>(result.code),
				result.result ? result.result->error_string.c_str() : "no result");
		}
		goalCompletionCallback(sequence, succeeded);
	};
	trajectoryActionClient->async_send_goal(goal, options);
}

void Controller::goalCompletionCallback(unsigned int goalSequence, bool succeeded)
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	if (goalSequence != trajectoryGoalSequence)
	{
		return; // Result of an older goal (cancelled or replaced)
	}

	if (!succeeded)
	{
		// The arm is somewhere along the way: look again and replan from there
		state = WAITING_FOR_POSE;
		return;
	}
	if (openLoopMode)
	{
		state = FINISHED_EXECUTION;
		return;
	}
	if (state == INTERIM_EXECUTION)
	{
		// Evaluated with the first object pose that is newer than the end of the move (see setCurrentObjectPose)
		awaitingFreshPose = true;
		freshPoseAfter = node->now() + rclcpp::Duration::from_seconds(settleTime);
		state = WAITING_FOR_POSE;
	}
	else if (state == FINAL_EXECUTION)
	{
		state = FINISHED_EXECUTION;
	}
	else if (state == SEARCHING_FOR_OBJECT)
	{
		state = WAITING_FOR_POSE;
	}
}

bool Controller::readyForFinalSequence()
{
	if (operationType == STOP)
	{
		return false;
	}
	// When following a moving object the arm never enters the final sequence
	if (operationType == INSPECT_NON_STATIONARY)
	{
		return false;
	}
	return isInterimPoseReached(estimateDistanceToObject(), estimateOrientationError());
}

bool Controller::isInterimPoseReached(float distanceToObject, float angleErrorInPose)
{
	double poseError = std::abs(distanceToObject - inspectionDistance);
	angleErrorInPose = std::abs(angleErrorInPose);
	if (distanceToObject >= 0 && poseError < inspectionPoseErrorMargin && angleErrorInPose < inspectionOrientationErrorMargin)
	{
		writeToLog("Inspection Position Reached With Pose error (threshold): " + std::to_string(poseError) + " (" + std::to_string(inspectionPoseErrorMargin) +
			") and Orientation Error (threshold): " + std::to_string(angleErrorInPose) + " (" + std::to_string(inspectionOrientationErrorMargin) + ") ");
		interimPoseReached = true;
		return true;
	}
	return false;
}

void Controller::performFinalSequence()
{
	if (operationType == TOUCH)
	{
		if (!calculateFinalTargetPose(inspectionDistance))
		{
			abandonPlanning();
			return;
		}
		planTrajectory();
		if (state == FINAL_EXECUTION)
		{
			sendGoal();
		}
	}
	else
	{
		// Inspection ends at the inspection pose, there is nothing more to do
		state = FINISHED_EXECUTION;
	}
}

void Controller::stop()
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	++trajectoryGoalSequence;
	trajectoryActionClient->async_cancel_all_goals();
	state = WAITING_FOR_POSE;
	finalSequence = false;
}

Controller::State Controller::getState()
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	return state;
}

bool Controller::isTaskComplete()
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	if (state == FINISHED_EXECUTION)
	{
		return true;
	}
	return operationType == INSPECT_STATIONARY && interimPoseReached;
}

float Controller::estimateDistanceToObject()
{
	bool found = true;
	auto t = findRequiredTransform(gripperFrameId, POIFrameId, *tfBuffer, &found);
	if (!found)
	{
		return -1;
	}
	return std::sqrt(std::pow(t.transform.translation.x, 2) + std::pow(t.transform.translation.y, 2) + std::pow(t.transform.translation.z, 2));
}

float Controller::estimateOrientationError()
{
	geometry_msgs::msg::Vector3Stamped gripperVec, objectVec;
	bool tfOk = true;
	computeGripperAndObjectVectors(gripperVec, objectVec, &tfOk);
	if (!tfOk)
	{
		return M_PI; // Unknown, never counts as reached
	}
	tf2::Vector3 gripper(gripperVec.vector.x, gripperVec.vector.y, gripperVec.vector.z);
	tf2::Vector3 object(objectVec.vector.x, objectVec.vector.y, objectVec.vector.z);
	return tf2::tf2Angle(gripper, object);
}

std::vector<std::string> Controller::getControllerInformationLines()
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	std::vector<std::string> lines;
	float distance = estimateDistanceToObject();
	lines.push_back("Controller State: " + getStateString());
	lines.push_back(distance < 0 ? "Estimate Distance To The Object: unknown"
	                             : "Estimate Distance To The Object: " + std::to_string(distance * 1000) + "mm");
	if (distance >= 0)
	{
		lines.push_back("Estimated Orientation Offset From Target Orientation: " + std::to_string(std::round(rtod(estimateOrientationError()) * 100) / 100) + " deg");
	}
	lines.push_back("Operation Type: " + getOperationTypeString());
	lines.push_back("Trajectory Truncation Type: " + getTruncationTypeString());
	return lines;
}

void Controller::updateTargetPoseMarker(const geometry_msgs::msg::PoseStamped& pose)
{
	targetPoseMarker.header.frame_id = pose.header.frame_id;
	targetPoseMarker.ns = "dexterity";
	targetPoseMarker.id = 20;
	targetPoseMarker.type = visualization_msgs::msg::Marker::ARROW;
	targetPoseMarker.action = visualization_msgs::msg::Marker::ADD;
	targetPoseMarker.pose = pose.pose;
	targetPoseMarker.scale.x = 0.3;
	targetPoseMarker.scale.y = 0.06;
	targetPoseMarker.scale.z = 0.03;
	targetPoseMarker.color.a = 1;
	targetPoseMarker.color.r = 0.7;
	targetPoseMarker.color.b = 0.3;
}

void Controller::updateVectorMarkers(const geometry_msgs::msg::Vector3Stamped& objectVector, const geometry_msgs::msg::Vector3Stamped& gripperVector)
{
	auto makeArrow = [](visualization_msgs::msg::Marker& marker, int id, const geometry_msgs::msg::Vector3Stamped& vector, double shaftDiameter)
	{
		geometry_msgs::msg::Point start, end;
		end.x = vector.vector.x;
		end.y = vector.vector.y;
		end.z = vector.vector.z;
		marker.header.frame_id = vector.header.frame_id;
		marker.ns = "dexterity";
		marker.id = id;
		marker.type = visualization_msgs::msg::Marker::ARROW;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.points = {start, end};
		marker.pose.orientation.w = 1;
		marker.color.g = 1;
		marker.color.a = 0.5;
		marker.scale.x = shaftDiameter;
		marker.scale.y = 0.06;
		marker.scale.z = 0.03;
	};
	makeArrow(objectVectorMarker, 3, objectVector, 0.03);
	makeArrow(gripperVectorMarker, 4, gripperVector, 0.04);
}

bool Controller::fillMarker(visualization_msgs::msg::MarkerArray& markerArray)
{
	std::lock_guard<std::recursive_mutex> lock(mutex);
	bool added = false;
	for (const auto* marker : {&targetPoseMarker, &objectVectorMarker, &gripperVectorMarker})
	{
		if (!marker->header.frame_id.empty())
		{
			markerArray.markers.push_back(*marker);
			added = true;
		}
	}
	return added;
}

std::string Controller::getStateString() const
{
	const char* states[] = {"WAITING_FOR_POSE", "SEARCHING_FOR_OBJECT", "INTERIM_PLANNING", "INTERIM_EXECUTION", "EVALUATION", "FINAL_PLANNING", "FINAL_EXECUTION", "FINISHED_EXECUTION"};
	return states[state];
}

std::string Controller::getOperationTypeString() const
{
	const char* types[] = {"INSPECT_STATIONARY", "INSPECT_NON_STATIONARY", "TOUCH", "GRASP", "STOP"};
	return types[operationType];
}

std::string Controller::getTruncationTypeString() const
{
	const char* types[] = {"HALF_TRAJECTORY", "QUARTER_TRAJECTORY", "STEP_BY_STEP"};
	return types[truncationType];
}

void Controller::writeToLog(const std::string& line) const
{
	std::ofstream logFile(logFilename, std::ios::app);
	if (!logFile.is_open())
	{
		RCLCPP_WARN(node->get_logger(), "Failed to open log file: %s", logFilename.c_str());
		return;
	}
	logFile << line << std::endl;
}
