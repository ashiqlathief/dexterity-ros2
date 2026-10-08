/*
 * HelpFunctions.hpp
 *
 * Frame conversion helpers (ported from help_functions.h of the ROS 1 node).
 */

#ifndef DEXTERITY_HELPFUNCTIONS_HPP_
#define DEXTERITY_HELPFUNCTIONS_HPP_

#include <cmath>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

inline constexpr double dtor(double angle) { return angle / 180.0 * M_PI; }
inline constexpr double rtod(double angle) { return angle * 180.0 / M_PI; }

// Expresses a pose / point in target_frame. On a failed lookup the identity transform is used and *success (if given)
// is set to false. *success is never set to true, so one flag can be passed through several calls.
geometry_msgs::msg::PoseStamped changeBaseFrame(const std::string& targetFrame, const geometry_msgs::msg::PoseStamped& pose, tf2_ros::Buffer& tfBuffer, bool* success = nullptr);
geometry_msgs::msg::PointStamped changeBaseFramePoint(const std::string& targetFrame, const geometry_msgs::msg::PointStamped& point, tf2_ros::Buffer& tfBuffer, bool* success = nullptr);

// Transform that maps sourceFrame coordinates into targetFrame. Identity (and success = false) if the lookup fails.
geometry_msgs::msg::TransformStamped findRequiredTransform(const std::string& targetFrame, const std::string& sourceFrame, tf2_ros::Buffer& tfBuffer, bool* success = nullptr);

geometry_msgs::msg::PoseStamped findPoseWRTFrame(const std::string& frame, const geometry_msgs::msg::PoseStamped& pose, tf2_ros::Buffer& tfBuffer, bool* success = nullptr);

std::string getCurrentDateTimeString();

#endif /* DEXTERITY_HELPFUNCTIONS_HPP_ */
