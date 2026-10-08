/*
 * HelpFunctions.cpp
 */

#include <dexterity/HelpFunctions.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
const rclcpp::Logger logger = rclcpp::get_logger("dexterity.tf");
rclcpp::Clock throttleClock(RCL_STEADY_TIME);

geometry_msgs::msg::TransformStamped identity(const std::string& targetFrame, const std::string& sourceFrame)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.frame_id = targetFrame;
	t.child_frame_id = sourceFrame;
	t.transform.rotation.w = 1;
	return t;
}
}

geometry_msgs::msg::TransformStamped findRequiredTransform(const std::string& targetFrame, const std::string& sourceFrame, tf2_ros::Buffer& tfBuffer, bool* success)
{
	try
	{
		auto t = tfBuffer.lookupTransform(targetFrame, sourceFrame, tf2::TimePointZero);
		return t;
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_WARN_THROTTLE(logger, throttleClock, 2000, "Lookup transform failed between %s and %s: %s", sourceFrame.c_str(), targetFrame.c_str(), ex.what());
		if (success) *success = false;
		return identity(targetFrame, sourceFrame);
	}
}

geometry_msgs::msg::PoseStamped changeBaseFrame(const std::string& targetFrame, const geometry_msgs::msg::PoseStamped& pose, tf2_ros::Buffer& tfBuffer, bool* success)
{
	geometry_msgs::msg::PoseStamped transformed;
	auto t = findRequiredTransform(targetFrame, pose.header.frame_id, tfBuffer, success);
	tf2::doTransform(pose.pose, transformed.pose, t);
	transformed.header.frame_id = targetFrame;
	return transformed;
}

geometry_msgs::msg::PointStamped changeBaseFramePoint(const std::string& targetFrame, const geometry_msgs::msg::PointStamped& point, tf2_ros::Buffer& tfBuffer, bool* success)
{
	geometry_msgs::msg::PointStamped transformed;
	auto t = findRequiredTransform(targetFrame, point.header.frame_id, tfBuffer, success);
	tf2::doTransform(point, transformed, t);
	transformed.header.frame_id = targetFrame;
	return transformed;
}

geometry_msgs::msg::PoseStamped findPoseWRTFrame(const std::string& frame, const geometry_msgs::msg::PoseStamped& pose, tf2_ros::Buffer& tfBuffer, bool* success)
{
	geometry_msgs::msg::PoseStamped transformed;
	auto t = findRequiredTransform(frame, pose.header.frame_id, tfBuffer, success);
	tf2::doTransform(pose, transformed, t);
	return transformed;
}

std::string getCurrentDateTimeString()
{
	auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::ostringstream stream;
	stream << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
	return stream.str();
}
