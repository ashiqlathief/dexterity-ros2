/*
 * ObjectOfInterest.hpp
 */

#ifndef DEXTERITY_OBJECTOFINTEREST_HPP_
#define DEXTERITY_OBJECTOFINTEREST_HPP_

#include <array>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <dexterity/ObjectPose.hpp>

class ObjectOfInterest
{
	public:
		// Selects the point of interest (POI) on the object, relative to the object origin.
		// "origin" uses the origin itself (e.g. the AprilTag cube); the other sides are the GET Lab pipestar offsets.
		void setSide(const std::string& side);
		void setPose(ObjectPose& pose, const builtin_interfaces::msg::Time& stamp);
		ObjectPose getPose() const { return objectPose; }
		bool isCurrentPoseValid() const { return objectPose.poseValidity; }

	private:
		ObjectPose objectPose;
		std::array<double, 6> POITransform {0, 0, 0, 0, 0, 0}; // x y z roll pitch yaw
		void updateObjectCoordinateFrame(const builtin_interfaces::msg::Time& stamp);
};

#endif /* DEXTERITY_OBJECTOFINTEREST_HPP_ */
