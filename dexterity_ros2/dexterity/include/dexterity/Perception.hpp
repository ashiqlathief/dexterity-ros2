/*
 * Perception.hpp
 *
 * Runs the selected tracker on every camera image and keeps the latest valid object pose.
 */

#ifndef DEXTERITY_PERCEPTION_HPP_
#define DEXTERITY_PERCEPTION_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <dexterity/ObjectOfInterest.hpp>
#include <dexterity/Settings.hpp>
#include <dexterity/Tracker.hpp>

class Perception
{
	public:
		Perception(rclcpp::Node::SharedPtr node, const DexteritySettings& settings);
		ObjectPose getPose() const { return objectOfInterest.getPose(); }
		void setCameraProperties(const sensor_msgs::msg::CameraInfo& cameraInfo);
		bool isInitialized() const { return camParamsSet; }
		void updateImage(const cv::Mat& image, const builtin_interfaces::msg::Time& stamp);
		void displayTrackingInformation(std::vector<std::string> textLines);
		bool fillMarker(visualization_msgs::msg::Marker& marker) const;

	private:
		rclcpp::Node::SharedPtr node;
		TrackerPtr tracker;
		ObjectOfInterest objectOfInterest;
		bool camParamsSet = false;
		double minAccuracyThreshold;

		bool checkPoseValidity(ObjectPose& pose) const;
};

#endif /* DEXTERITY_PERCEPTION_HPP_ */
