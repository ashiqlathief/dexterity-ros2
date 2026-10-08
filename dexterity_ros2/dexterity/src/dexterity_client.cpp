/*
 * dexterity_client.cpp
 *
 * Sends a goal to the dexterity action server and waits for the result.
 *
 *   ros2 run dexterity dexterity_client <command> [sim] [tracker_mode] [tracker] [object] [gripper_frame]
 *                                                 [truncation] [open_loop] [orientation_lock] [side]
 *
 *   command        Inspection | Follow | Touch | Grasp | Stop | Reset
 *   sim            1 | 0
 *   tracker_mode   hybrid | me | klt
 *   tracker        visp | icg | m3t
 *   object         pipestar | pipestar_og | cube_at10 | cube_rl2 | <file>.cao
 *   gripper_frame  frame id of the gripper (+X pointing out of the gripper)
 *   truncation     half_trajectory | quater_trajectory | step_by_step
 *   open_loop      0 | 1
 *   orientation_lock 0 | 1
 *   side           origin | top | front | right | left
 *
 * Arguments are positional as in ROS 1, but only the command is required: missing (or "-") arguments
 * keep the values from the node's parameter file. Example:
 *   ros2 run dexterity dexterity_client Inspection 1 hybrid visp cube_rl2 dexterity_tool_frame half_trajectory 0 0 origin
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dexterity_msgs/action/dexterity.hpp>

using DexterityAction = dexterity_msgs::action::Dexterity;
using namespace std::chrono_literals;

namespace
{
std::atomic<bool> interrupted {false};

int usage()
{
	std::cerr << "Usage: dexterity_client <Inspection|Follow|Touch|Grasp|Stop|Reset> [sim 1|0] [hybrid|me|klt] [visp|icg|m3t] "
	             "[object] [gripper_frame] [half_trajectory|quater_trajectory|step_by_step] [open_loop 0|1] [orientation_lock 0|1] "
	             "[origin|top|front|right|left]" << std::endl;
	return 1;
}

bool isSet(const std::vector<std::string>& args, size_t index)
{
	return index < args.size() && args[index] != "-";
}
}

int main(int argc, char** argv)
{
	// Own SIGINT handling, so Ctrl+C can still cancel the goal before shutting down
	rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
	std::signal(SIGINT, [](int) { interrupted = true; });
	std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
	if (args.size() < 2)
	{
		return usage();
	}

	static const std::map<std::string, std::string> commands {
		{"Inspection", "inspect_stationary"}, {"Follow", "inspect_non_stationary"}, {"Touch", "touch"},
		{"Grasp", "grasp"}, {"Stop", "stop"}, {"Reset", "reset"}, {"UpdateParameters", "update_parameters"}};
	static const std::map<std::string, std::string> objectModels {
		{"pipestar", "pipestar.cao"}, {"pipestar_og", "pipestar_og.cao"}, {"cube_at10", "cube_at10.cao"}, {"cube_rl2", "cube_at_rl2.cao"}};

	DexterityAction::Goal goal;
	auto command = commands.find(args[1]);
	if (command == commands.end())
	{
		std::cerr << "Unknown command: " << args[1] << std::endl;
		return usage();
	}
	goal.command = command->second;

	goal.simulator_mode = !isSet(args, 2) || args[2] == "1";
	if (isSet(args, 3)) goal.tracker_mode = args[3];
	if (isSet(args, 4)) goal.tracker_name = args[4];
	if (isSet(args, 5))
	{
		auto model = objectModels.find(args[5]);
		goal.object_model = model != objectModels.end() ? model->second : args[5];
	}
	if (isSet(args, 6)) goal.gripper_frame_id = args[6];
	if (isSet(args, 7)) goal.truncation_type = args[7];
	goal.open_loop_mode = isSet(args, 8) && args[8] == "1";
	goal.orientation_lock = isSet(args, 9) && args[9] == "1";
	if (isSet(args, 10)) goal.object_side = args[10];

	auto node = rclcpp::Node::make_shared("dexterity_client");
	auto client = rclcpp_action::create_client<DexterityAction>(node, "dexterity_action");
	RCLCPP_INFO(node->get_logger(), "Waiting for the dexterity action server...");
	if (!client->wait_for_action_server(30s))
	{
		RCLCPP_ERROR(node->get_logger(), "Action server dexterity_action not available");
		rclcpp::shutdown();
		return 1;
	}

	rclcpp_action::Client<DexterityAction>::SendGoalOptions options;
	options.feedback_callback = [&node](auto, const std::shared_ptr<const DexterityAction::Feedback> feedback)
	{
		if (feedback->percentage >= 0)
		{
			RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Distance gripper -> object: %.3f m", feedback->percentage);
		}
	};

	RCLCPP_INFO(node->get_logger(), "Sending goal: %s", goal.command.c_str());
	auto goalFuture = client->async_send_goal(goal, options);
	if (rclcpp::spin_until_future_complete(node, goalFuture, 10s) != rclcpp::FutureReturnCode::SUCCESS || !goalFuture.get())
	{
		RCLCPP_ERROR(node->get_logger(), "Goal was not accepted");
		rclcpp::shutdown();
		return 1;
	}

	// Ctrl+C cancels the task
	auto goalHandle = goalFuture.get();
	auto resultFuture = client->async_get_result(goalHandle);
	while (!interrupted)
	{
		if (rclcpp::spin_until_future_complete(node, resultFuture, 100ms) == rclcpp::FutureReturnCode::SUCCESS)
		{
			break;
		}
	}
	if (interrupted)
	{
		RCLCPP_INFO(node->get_logger(), "Cancelling the task...");
		auto cancelFuture = client->async_cancel_goal(goalHandle);
		rclcpp::spin_until_future_complete(node, cancelFuture, 2s);
		rclcpp::spin_until_future_complete(node, resultFuture, 2s);
		rclcpp::shutdown();
		return 0;
	}

	auto wrapped = resultFuture.get();
	const char* outcome = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? "succeeded"
	                    : wrapped.code == rclcpp_action::ResultCode::CANCELED ? "cancelled" : "aborted";
	RCLCPP_INFO(node->get_logger(), "Goal %s (success = %s)", outcome, wrapped.result && wrapped.result->success ? "true" : "false");
	rclcpp::shutdown();
	return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 1;
}
