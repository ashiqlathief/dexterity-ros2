/*
 * Controller.hpp
 *
 * Closed loop hand-eye controller. For every valid object pose it plans a motion of the gripper
 * towards the inspection pose in front of the object (distance inspectionDistance along the Z axis of
 * object_frame, gripper +X pointing at the object), executes a truncated part of the trajectory and
 * looks at the object again, until the inspection pose is reached within the error margins.
 */

#ifndef DEXTERITY_CONTROLLER_HPP_
#define DEXTERITY_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <dexterity_msgs/msg/dexterity_event.hpp>
#include <dexterity/ObjectPose.hpp>
#include <dexterity/PlanningSupervisor.hpp>
#include <dexterity/Settings.hpp>

class Controller
{
	public:
		using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
		using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

		enum State {WAITING_FOR_POSE, SEARCHING_FOR_OBJECT, INTERIM_PLANNING, INTERIM_EXECUTION, EVALUATION, FINAL_PLANNING, FINAL_EXECUTION, FINISHED_EXECUTION};
		enum OperationType {INSPECT_STATIONARY, INSPECT_NON_STATIONARY, TOUCH, GRASP, STOP};
		enum TruncationType {HALF_TRAJECTORY, QUARTER_TRAJECTORY, STEP_BY_STEP};

		Controller(rclcpp::Node::SharedPtr node, std::shared_ptr<MoveGroupInterface> moveGroup,
		           std::shared_ptr<tf2_ros::Buffer> tfBuffer, rclcpp::CallbackGroup::SharedPtr callbackGroup);

		void setParameters(const DexteritySettings& settings); // Also resets the state for a new run
		void setCurrentObjectPose(const ObjectPose& objectPose); // Call in WAITING_FOR_POSE only
		void stop(); // Cancels the running trajectory

		State getState();
		bool isTaskComplete();
		float estimateDistanceToObject(); // -1 if the object frame is not known yet
		std::vector<std::string> getControllerInformationLines();
		bool fillMarker(visualization_msgs::msg::MarkerArray& markerArray);
		// Task and planning events for evaluation tools (/dexterity/events)
		void publishEvent(dexterity_msgs::msg::DexterityEvent event);

	private:
		rclcpp::Node::SharedPtr node;
		std::shared_ptr<MoveGroupInterface> moveGroupInterface;
		std::shared_ptr<tf2_ros::Buffer> tfBuffer;
		tf2_ros::TransformBroadcaster tfBroadcaster;
		rclcpp::Publisher<dexterity_msgs::msg::DexterityEvent>::SharedPtr eventPublisher;
		std::string trajectoryActionName;
		rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectoryActionClient;
		std::recursive_mutex mutex; // The trajectory result callback runs in another thread than the image callback

		PlanningSupervisor supervisor;

		// Parameters
		OperationType operationType = INSPECT_STATIONARY;
		bool openLoopMode = false;
		bool orientationLock = false;
		double inspectionDistance = 0;
		double inspectionPoseErrorMargin = 0;        // m
		double inspectionOrientationErrorMargin = 0; // rad
		std::string gripperFrameId;
		std::string baseFrame;
		TruncationType truncationType = HALF_TRAJECTORY;
		std::string logFilename = "controller_log.txt";

		// State
		State state = WAITING_FOR_POSE;
		bool finalSequence = false;
		bool interimPoseReached = false;
		int numberOfPlanningSuccesses = 0;
		unsigned int trajectoryGoalSequence = 0; // Results of older trajectory goals are ignored
		bool awaitingFreshPose = false;          // After a move: the next object pose must come from a newer image
		rclcpp::Time freshPoseAfter{0, 0, RCL_ROS_TIME};
		static constexpr double maxJointChange = 2.5; // rad, larger IK solutions are on the other side of the arm
		static constexpr double settleTime = 0.2; // s the arm gets to settle before the image counts
		MoveGroupInterface::Plan plan;
		geometry_msgs::msg::PoseStamped interimTargetPoseWRTGripper;
		geometry_msgs::msg::PoseStamped finalTargetPoseWRTGripper;
		geometry_msgs::msg::PoseStamped searchTargetPose;

		// Visualization markers for debugging
		visualization_msgs::msg::Marker targetPoseMarker, objectVectorMarker, gripperVectorMarker;

		bool calculateInterimTargetPose(const geometry_msgs::msg::TransformStamped& currentObjectFrame);
		bool calculateFinalTargetPose(double perpendicularDistance); // false if a transform is missing
		tf2::Quaternion findShortestRotation(bool* tfOk);
		void computeGripperAndObjectVectors(geometry_msgs::msg::Vector3Stamped& gripperVector, geometry_msgs::msg::Vector3Stamped& objectVector, bool* tfOk);
		void planTrajectory();
		void abandonPlanning();
		bool solveIKLocally(moveit::core::RobotState& state, const moveit::core::JointModelGroup* group, const Eigen::Isometry3d& target);
		bool setJointGoalNearCurrent(const geometry_msgs::msg::PoseStamped& poseWRTBase);
		void truncateTrajectory();
		void sendGoal();
		void goalCompletionCallback(unsigned int goalSequence, bool succeeded);
		bool readyForFinalSequence();
		float estimateOrientationError();
		bool isInterimPoseReached(float distanceToObject, float angleErrorInPose);
		void performFinalSequence();
		void updateTargetPoseMarker(const geometry_msgs::msg::PoseStamped& pose);
		void updateVectorMarkers(const geometry_msgs::msg::Vector3Stamped& objectVector, const geometry_msgs::msg::Vector3Stamped& gripperVector);

		std::string getStateString() const;
		std::string getOperationTypeString() const;
		std::string getTruncationTypeString() const;
		void writeToLog(const std::string& text) const;
};

#endif /* DEXTERITY_CONTROLLER_HPP_ */
