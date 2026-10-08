/*
 * Perception.cpp
 */

#include <dexterity/Perception.hpp>

#include <stdexcept>

#include <dexterity/ExternalPoseTracker.hpp>
#include <dexterity/VISPTracker.hpp>

Perception::Perception(rclcpp::Node::SharedPtr node, const DexteritySettings& settings)
	: node(node), minAccuracyThreshold(settings.perception.minAccuracyThreshold)
{
	const auto& name = settings.perception.trackerName;
	if (name == "visp")
	{
		tracker = std::make_shared<VISPTracker>(settings.perception, settings.robot.cameraOpticalFrame);
	}
	else if (name == "icg" || name == "m3t")
	{
		tracker = std::make_shared<ExternalPoseTracker>(node, settings.perception.externalPoseTopic);
	}
	else
	{
		throw std::runtime_error("Unknown tracker '" + name + "'. Available trackers: visp, icg, m3t");
	}
	objectOfInterest.setSide(settings.perception.objectSide);
	RCLCPP_INFO(node->get_logger(), "Perception ready: tracker %s, minimum accuracy %.0f%%", name.c_str(), minAccuracyThreshold);
}

void Perception::setCameraProperties(const sensor_msgs::msg::CameraInfo& cameraInfo)
{
	tracker->setCameraProperties(cameraInfo);
	camParamsSet = true;
}

void Perception::updateImage(const cv::Mat& image, const builtin_interfaces::msg::Time& stamp)
{
	ObjectPose pose;
	tracker->updateImage(image, pose);
	if (checkPoseValidity(pose))
	{
		tracker->requestNewTrack();
	}
	objectOfInterest.setPose(pose, stamp);
}

/*
 * Returns true if a new track is required (accuracy below the threshold for an updated pose).
 * Also sets the validity flag of the pose.
 */
bool Perception::checkPoseValidity(ObjectPose& pose) const
{
	if (pose.accuracy == -1) // Pose was not updated, disregard it
	{
		pose.poseValidity = false;
		return false;
	}
	if (pose.accuracy >= minAccuracyThreshold && pose.accuracy < 101)
	{
		pose.poseValidity = true;
		return false;
	}
	pose.poseValidity = false;
	return true;
}

void Perception::displayTrackingInformation(std::vector<std::string> textLines)
{
	ObjectPose pose = getPose();
	textLines.push_back("Pose Accuracy: " + std::to_string(pose.accuracy) + "%");
	textLines.push_back(std::string("Valid Pose Available: ") + (pose.poseValidity ? "yes" : "no"));
	tracker->displayText(textLines);
}

bool Perception::fillMarker(visualization_msgs::msg::Marker& marker) const
{
	if (!objectOfInterest.isCurrentPoseValid())
	{
		return false;
	}
	ObjectPose pose = getPose();
	marker.header.frame_id = pose.poseStamped.header.frame_id;
	marker.ns = "dexterity";
	marker.id = 1;
	marker.type = visualization_msgs::msg::Marker::CUBE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose = pose.poseStamped.pose;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 0.5;
	marker.color.b = 1.0;
	return true;
}
