/*
 * PlanningSupervisor.cpp
 */

#include <dexterity/PlanningSupervisor.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Tilts the end-effector up by pitchIncrement to look for the object
geometry_msgs::msg::PoseStamped PlanningSupervisor::lookUpForObject(const geometry_msgs::msg::PoseStamped& currentPose) const
{
	geometry_msgs::msg::PoseStamped searchTargetPose = currentPose;
	const auto& o = currentPose.pose.orientation;
	double roll, pitch, yaw;
	tf2::Matrix3x3(tf2::Quaternion(o.x, o.y, o.z, o.w)).getRPY(roll, pitch, yaw);
	pitch += pitchIncrement;
	tf2::Quaternion qTarget;
	qTarget.setEuler(yaw, pitch, roll); // Same as the tf2::Quaternion(yaw, pitch, roll) constructor used in ROS 1
	searchTargetPose.pose.orientation.w = qTarget.w();
	searchTargetPose.pose.orientation.x = qTarget.x();
	searchTargetPose.pose.orientation.y = qTarget.y();
	searchTargetPose.pose.orientation.z = qTarget.z();
	return searchTargetPose;
}
