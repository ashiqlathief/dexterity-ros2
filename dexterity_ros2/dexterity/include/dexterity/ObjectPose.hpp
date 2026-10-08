/*
 * ObjectPose.hpp
 */

#ifndef DEXTERITY_OBJECTPOSE_HPP_
#define DEXTERITY_OBJECTPOSE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

struct ObjectPose
{
	geometry_msgs::msg::PoseStamped poseStamped;          // Pose of the object origin in the camera optical frame
	geometry_msgs::msg::TransformStamped coordinateFrame;  // camera -> object_origin
	geometry_msgs::msg::TransformStamped POIFrame;         // object_origin -> object_frame (Z axis = direction of approach)
	double accuracy = -1;                                  // In percent, (90 - projection error) / 90 * 100. -1 = pose not updated
	bool poseValidity = false;
};

#endif /* DEXTERITY_OBJECTPOSE_HPP_ */
