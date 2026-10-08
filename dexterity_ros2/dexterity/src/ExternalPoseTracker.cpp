/*
 * ExternalPoseTracker.cpp
 */

#include <dexterity/ExternalPoseTracker.hpp>

ExternalPoseTracker::ExternalPoseTracker(rclcpp::Node::SharedPtr node, const std::string& poseTopic)
{
	std::weak_ptr<LatestPose> weakLatest = latest;
	poseSubscriber = node->create_subscription<dexterity_msgs::msg::EstimatedPose>(poseTopic, 1,
		[weakLatest](const dexterity_msgs::msg::EstimatedPose& msg)
		{
			auto shared = weakLatest.lock();
			if (!shared)
			{
				return;
			}
			std::lock_guard<std::mutex> lock(shared->mutex);
			shared->pose.poseStamped = msg.pose_stamped;
			shared->pose.poseValidity = msg.success;
			shared->pose.accuracy = msg.success ? 100 : -1; // These trackers do not report a projection error
		});
	RCLCPP_INFO(node->get_logger(), "Using poses from an external tracker on %s", poseTopic.c_str());
}

void ExternalPoseTracker::updateImage(const cv::Mat&, ObjectPose& pose)
{
	std::lock_guard<std::mutex> lock(latest->mutex);
	pose = latest->pose;
}
