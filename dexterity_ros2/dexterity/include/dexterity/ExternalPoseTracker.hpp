/*
 * ExternalPoseTracker.hpp
 *
 * Uses poses from a separate tracker node (ICG or M3T pose estimation), published as
 * dexterity_msgs/EstimatedPose. Replaces ICGTracker and M3TTracker of the ROS 1 node, which were identical.
 */

#ifndef DEXTERITY_EXTERNALPOSETRACKER_HPP_
#define DEXTERITY_EXTERNALPOSETRACKER_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <dexterity_msgs/msg/estimated_pose.hpp>
#include <dexterity/Tracker.hpp>

class ExternalPoseTracker : public Tracker
{
	public:
		ExternalPoseTracker(rclcpp::Node::SharedPtr node, const std::string& poseTopic);
		void setCameraProperties(const sensor_msgs::msg::CameraInfo&) override {}
		void updateImage(const cv::Mat& image, ObjectPose& pose) override;
		void requestNewTrack() override {}
		void displayText(const std::vector<std::string>&) override {}

	private:
		// Shared with the subscription callback, so a callback that runs while the tracker is replaced stays safe
		struct LatestPose
		{
			std::mutex mutex;
			ObjectPose pose;
		};
		std::shared_ptr<LatestPose> latest = std::make_shared<LatestPose>();
		rclcpp::Subscription<dexterity_msgs::msg::EstimatedPose>::SharedPtr poseSubscriber;
};

#endif /* DEXTERITY_EXTERNALPOSETRACKER_HPP_ */
