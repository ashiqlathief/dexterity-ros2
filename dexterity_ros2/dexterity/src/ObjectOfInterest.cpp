/*
 * ObjectOfInterest.cpp
 */

#include <dexterity/ObjectOfInterest.hpp>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>

void ObjectOfInterest::setSide(const std::string& side)
{
	if (side == "origin")
	{
		POITransform = {0, 0, 0, 0, 0, 0};
	}
	else if (side == "top") // pipe 1
	{
		POITransform = {-0.017, 0.1949, 0.0088, 0, -0.92439, 0};
	}
	else if (side == "front")
	{
		POITransform = {-0.051, 0, -0.023, 0, -0.785, 0};
	}
	else if (side == "front_level") // same point as "front", seen horizontally (the UR5e cannot look down from 45 deg at this height)
	{
		POITransform = {-0.051, 0, -0.023, 0, -1.5707963, 0};
	}
	else if (side == "front_tilt") // same point as "front", seen from 16 deg above
	{
		POITransform = {-0.051, 0, -0.023, 0, -1.3, 0};
	}
	else if (side == "right") // pipe 2
	{
		POITransform = {-0.0041, -0.1157, 0.1558, 0, -0.974, 0};
	}
	else if (side == "left")
	{
		POITransform = {-0.00983, 0.07009, -0.0494, 0, -0.850, 0};
	}
	else
	{
		RCLCPP_WARN(rclcpp::get_logger("dexterity.perception"), "Unknown object side '%s', using the object origin", side.c_str());
		POITransform = {0, 0, 0, 0, 0, 0};
	}
}

void ObjectOfInterest::setPose(ObjectPose& pose, const builtin_interfaces::msg::Time& stamp)
{
	objectPose = pose;
	if (objectPose.poseValidity)
	{
		updateObjectCoordinateFrame(stamp);
	}
}

void ObjectOfInterest::updateObjectCoordinateFrame(const builtin_interfaces::msg::Time& stamp)
{
	// camera -> object_origin (pose returned by the tracker)
	auto& frame = objectPose.coordinateFrame;
	frame.header.stamp = stamp;
	frame.header.frame_id = objectPose.poseStamped.header.frame_id;
	frame.child_frame_id = "object_origin";
	frame.transform.translation.x = objectPose.poseStamped.pose.position.x;
	frame.transform.translation.y = objectPose.poseStamped.pose.position.y;
	frame.transform.translation.z = objectPose.poseStamped.pose.position.z;
	frame.transform.rotation = objectPose.poseStamped.pose.orientation;

	// object_origin -> object_frame (point of interest)
	tf2::Quaternion qRotation;
	qRotation.setRPY(POITransform[3], POITransform[4], POITransform[5]);
	auto& poi = objectPose.POIFrame;
	poi.header.stamp = stamp;
	poi.header.frame_id = "object_origin";
	poi.child_frame_id = "object_frame";
	poi.transform.translation.x = POITransform[0];
	poi.transform.translation.y = POITransform[1];
	poi.transform.translation.z = POITransform[2];
	poi.transform.rotation.w = qRotation.w();
	poi.transform.rotation.x = qRotation.x();
	poi.transform.rotation.y = qRotation.y();
	poi.transform.rotation.z = qRotation.z();
}
