/*
 * DexterityNode.cpp
 *
 * ROS 2 node of the dexterity hand-eye controller.
 *
 *  - Action server "dexterity_action" (dexterity_msgs/action/Dexterity): starts a task
 *    (inspect_stationary, inspect_non_stationary, touch), or handles stop / reset / update_parameters.
 *    The goal succeeds when the task is complete; cancel it to stop.
 *  - Service "arm_home" (dexterity_msgs/srv/ArmHome): moves the arm to the MoveIt home state.
 *  - Camera image + camera info: every image goes through the perception module; when the
 *    controller waits for a pose it plans and executes the next motion.
 */

#include <memory>
#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <dexterity_msgs/action/dexterity.hpp>
#include <dexterity_msgs/srv/arm_home.hpp>

#include <dexterity/Controller.hpp>
#include <dexterity/Perception.hpp>
#include <dexterity/Settings.hpp>

using namespace std::chrono_literals;

class DexterityNode : public rclcpp::Node
{
	public:
		using DexterityAction = dexterity_msgs::action::Dexterity;
		using GoalHandle = rclcpp_action::ServerGoalHandle<DexterityAction>;
		using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

		explicit DexterityNode(const rclcpp::NodeOptions& options) : Node("dexterity", options)
		{
			settings.declareAndLoad(*this);
		}

		// Needs shared_from_this(), so it cannot be done in the constructor
		void init()
		{
			auto self = shared_from_this();

			tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
			tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, self, true);

			RCLCPP_INFO(get_logger(), "Connecting to MoveIt (planning group '%s')...", settings.robot.moveGroup.c_str());
			try
			{
				moveGroup = std::make_shared<MoveGroupInterface>(self, settings.robot.moveGroup, tfBuffer, rclcpp::Duration::from_seconds(60));
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error(std::string("MoveIt move_group is not available (is it running?): ") + e.what());
			}
			RCLCPP_INFO(get_logger(), "MoveIt connected. Planning frame: %s, end-effector: %s",
				moveGroup->getPlanningFrame().c_str(), moveGroup->getEndEffectorLink().c_str());

			// Separate callback groups so that long image processing does not block the other callbacks.
			// They are added to the executor explicitly (see addToExecutor), not automatically with the node.
			auto cameraGroup = createGroup();
			auto actionGroup = createGroup();
			auto trajectoryGroup = createGroup();
			auto serviceGroup = createGroup();

			controller = std::make_shared<Controller>(self, moveGroup, tfBuffer, trajectoryGroup);

			rclcpp::SubscriptionOptions cameraOptions;
			cameraOptions.callback_group = cameraGroup;
			imageSubscriber = create_subscription<sensor_msgs::msg::Image>(settings.robot.imageTopic, rclcpp::SensorDataQoS().keep_last(1),
				[this](sensor_msgs::msg::Image::ConstSharedPtr msg) { imageCallback(msg); }, cameraOptions);
			camInfoSubscriber = create_subscription<sensor_msgs::msg::CameraInfo>(settings.robot.cameraInfoTopic, rclcpp::SensorDataQoS(),
				[this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { std::lock_guard<std::mutex> lock(cameraInfoMutex); cameraInfo = msg; },
				cameraOptions);

			markerPublisher = create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

			homeService = create_service<dexterity_msgs::srv::ArmHome>("arm_home",
				[this](const std::shared_ptr<dexterity_msgs::srv::ArmHome::Request>, std::shared_ptr<dexterity_msgs::srv::ArmHome::Response> response)
				{
					{
						std::lock_guard<std::recursive_mutex> lock(runMutex);
						abortActiveGoal("arm reset requested");
					}
					response->success = moveHome();
					response->message = response->success ? "Arm moved to " + settings.robot.homeState : "Planning to the home state failed";
				},
				rclcpp::ServicesQoS(), serviceGroup);

			actionServer = rclcpp_action::create_server<DexterityAction>(self, "dexterity_action",
				[](const rclcpp_action::GoalUUID&, std::shared_ptr<const DexterityAction::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
				[](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
				[this](const std::shared_ptr<GoalHandle> goalHandle) { handleAccepted(goalHandle); },
				rcl_action_server_get_default_options(), actionGroup);

			// Feedback, cancel handling and completion check of the active goal
			feedbackTimer = create_wall_timer(200ms, [this]() { updateActiveGoal(); }, actionGroup);

			RCLCPP_INFO(get_logger(), "Dexterity ready. Camera: %s, trajectory action: %s",
				settings.robot.imageTopic.c_str(), settings.robot.trajectoryAction.c_str());
		}

		void addToExecutor(rclcpp::Executor& executor)
		{
			executor.add_node(get_node_base_interface());
			for (const auto& group : callbackGroups)
			{
				executor.add_callback_group(group, get_node_base_interface());
			}
		}

	private:
		std::vector<rclcpp::CallbackGroup::SharedPtr> callbackGroups;

		rclcpp::CallbackGroup::SharedPtr createGroup()
		{
			callbackGroups.push_back(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
			return callbackGroups.back();
		}

		DexteritySettings settings;
		std::shared_ptr<tf2_ros::Buffer> tfBuffer;
		std::shared_ptr<tf2_ros::TransformListener> tfListener;
		std::shared_ptr<MoveGroupInterface> moveGroup;
		std::shared_ptr<Controller> controller;
		std::unique_ptr<Perception> perception;

		rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camInfoSubscriber;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;
		rclcpp::Service<dexterity_msgs::srv::ArmHome>::SharedPtr homeService;
		rclcpp_action::Server<DexterityAction>::SharedPtr actionServer;
		rclcpp::TimerBase::SharedPtr feedbackTimer;

		std::mutex cameraInfoMutex;
		sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo;

		std::recursive_mutex runMutex; // Guards dexRun, perception, settings and activeGoal
		bool dexRun = false;
		std::shared_ptr<GoalHandle> activeGoal;

		void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
		{
			std::lock_guard<std::recursive_mutex> lock(runMutex);
			if (!dexRun || !perception)
			{
				return;
			}
			if (!perception->isInitialized())
			{
				std::lock_guard<std::mutex> infoLock(cameraInfoMutex);
				if (!cameraInfo)
				{
					RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera info on %s", settings.robot.cameraInfoTopic.c_str());
					return;
				}
				perception->setCameraProperties(*cameraInfo);
			}

			cv_bridge::CvImagePtr cvImage;
			try
			{
				cvImage = cv_bridge::toCvCopy(msg, "bgr8");
			}
			catch (const cv_bridge::Exception& e)
			{
				RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
				return;
			}

			try
			{
				perception->updateImage(cvImage->image, msg->header.stamp);
				if (controller->getState() == Controller::WAITING_FOR_POSE)
				{
					controller->setCurrentObjectPose(perception->getPose());
				}
				perception->displayTrackingInformation(controller->getControllerInformationLines());
				publishMarkers();
			}
			catch (const std::exception& e)
			{
				RCLCPP_ERROR(get_logger(), "Stopping the task: %s", e.what());
				abortActiveGoal(e.what());
			}
		}

		void publishMarkers()
		{
			visualization_msgs::msg::MarkerArray markers;
			visualization_msgs::msg::Marker objectMarker;
			if (perception->fillMarker(objectMarker))
			{
				markers.markers.push_back(objectMarker);
			}
			controller->fillMarker(markers);
			if (!markers.markers.empty())
			{
				markerPublisher->publish(markers);
			}
		}

		void publishTaskEvent(const std::string& event, const std::string& result = "")
		{
			dexterity_msgs::msg::DexterityEvent message;
			message.event = event;
			message.command = settings.controller.operationType;
			message.tracker_name = settings.perception.trackerName;
			message.object_model = settings.perception.objectModelFilename;
			message.object_side = settings.perception.objectSide;
			message.inspection_distance = settings.controller.inspectionDistance;
			message.gripper_frame_id = settings.controller.gripperFrameId;
			message.result = result;
			controller->publishEvent(message);
		}

		void handleAccepted(const std::shared_ptr<GoalHandle>& goalHandle)
		{
			const auto goal = goalHandle->get_goal();
			RCLCPP_INFO(get_logger(), "Received goal: %s", goal->command.c_str());
			auto result = std::make_shared<DexterityAction::Result>();

			if (goal->command == "reset")
			{
				// Moving home blocks, so it runs in its own thread
				std::thread([this, goalHandle, result]()
				{
					{
						std::lock_guard<std::recursive_mutex> lock(runMutex);
						abortActiveGoal("reset requested");
					}
					result->success = moveHome();
					result->success ? goalHandle->succeed(result) : goalHandle->abort(result);
				}).detach();
				return;
			}

			std::lock_guard<std::recursive_mutex> lock(runMutex);
			if (goal->command == "stop")
			{
				abortActiveGoal("stop requested");
				result->success = true;
				goalHandle->succeed(result);
				return;
			}
			if (goal->command == "update_parameters")
			{
				settings.applyGoal(*goal);
				result->success = true;
				goalHandle->succeed(result);
				return;
			}

			abortActiveGoal("replaced by a new goal");
			settings.applyGoal(*goal);
			try
			{
				controller->setParameters(settings);
				perception = std::make_unique<Perception>(shared_from_this(), settings);
				activeGoal = goalHandle;
				dexRun = true;
				RCLCPP_INFO(get_logger(), "Task started: %s with the %s tracker, object model %s",
					settings.controller.operationType.c_str(), settings.perception.trackerName.c_str(), settings.perception.objectModelFilename.c_str());
				publishTaskEvent(dexterity_msgs::msg::DexterityEvent::TASK_STARTED);
			}
			catch (const std::exception& e)
			{
				RCLCPP_ERROR(get_logger(), "Could not start the task: %s", e.what());
				result->success = false;
				goalHandle->abort(result);
			}
		}

		void updateActiveGoal()
		{
			std::lock_guard<std::recursive_mutex> lock(runMutex);
			if (!activeGoal)
			{
				return;
			}
			auto result = std::make_shared<DexterityAction::Result>();
			if (activeGoal->is_canceling())
			{
				RCLCPP_INFO(get_logger(), "Task cancelled");
				stopRun();
				publishTaskEvent(dexterity_msgs::msg::DexterityEvent::TASK_FINISHED, "cancelled");
				result->success = false;
				activeGoal->canceled(result);
				activeGoal.reset();
				return;
			}

			auto feedback = std::make_shared<DexterityAction::Feedback>();
			feedback->percentage = controller->estimateDistanceToObject(); // Distance gripper -> object in m (-1 = unknown), as in ROS 1
			activeGoal->publish_feedback(feedback);

			if (controller->isTaskComplete())
			{
				RCLCPP_INFO(get_logger(), "Task complete");
				dexRun = false;
				publishTaskEvent(dexterity_msgs::msg::DexterityEvent::TASK_FINISHED, "succeeded");
				result->success = true;
				activeGoal->succeed(result);
				activeGoal.reset();
			}
		}

		void abortActiveGoal(const std::string& reason)
		{
			stopRun();
			if (activeGoal && activeGoal->is_active())
			{
				RCLCPP_INFO(get_logger(), "Aborting the running task: %s", reason.c_str());
				publishTaskEvent(dexterity_msgs::msg::DexterityEvent::TASK_FINISHED, "aborted: " + reason);
				auto result = std::make_shared<DexterityAction::Result>();
				result->success = false;
				activeGoal->abort(result);
			}
			activeGoal.reset();
		}

		void stopRun()
		{
			if (dexRun)
			{
				dexRun = false;
				controller->stop();
				moveGroup->stop();
			}
		}

		bool moveHome()
		{
			moveGroup->setStartStateToCurrentState();
			moveGroup->setNamedTarget(settings.robot.homeState);
			MoveGroupInterface::Plan plan;
			if (!static_cast<bool>(moveGroup->plan(plan)))
			{
				RCLCPP_ERROR(get_logger(), "Planning to the home state '%s' failed", settings.robot.homeState.c_str());
				return false;
			}
			RCLCPP_INFO(get_logger(), "Moving to the home state '%s'", settings.robot.homeState.c_str());
			return static_cast<bool>(moveGroup->execute(plan));
		}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	// MoveIt reads robot_description, robot_description_semantic, kinematics etc. from parameter overrides
	auto node = std::make_shared<DexterityNode>(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
	try
	{
		node->init();
	}
	catch (const std::exception& e)
	{
		RCLCPP_FATAL(node->get_logger(), "%s", e.what());
		rclcpp::shutdown();
		return 1;
	}
	rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
	node->addToExecutor(executor);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
