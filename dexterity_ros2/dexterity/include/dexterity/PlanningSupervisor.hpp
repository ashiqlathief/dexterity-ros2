/*
 * PlanningSupervisor.hpp
 *
 * Optional search behaviour when no object pose is available (disabled by default, as in ROS 1).
 */

#ifndef DEXTERITY_PLANNINGSUPERVISOR_HPP_
#define DEXTERITY_PLANNINGSUPERVISOR_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>

class PlanningSupervisor
{
	public:
		bool isEnabled = false;
		geometry_msgs::msg::PoseStamped lookUpForObject(const geometry_msgs::msg::PoseStamped& currentPose) const;

	private:
		double pitchIncrement = 0.087;
};

#endif /* DEXTERITY_PLANNINGSUPERVISOR_HPP_ */
