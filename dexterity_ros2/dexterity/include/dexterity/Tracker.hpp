/*
 * Tracker.hpp
 *
 * Interface of the object trackers used by the perception module.
 */

#ifndef DEXTERITY_TRACKER_HPP_
#define DEXTERITY_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <dexterity/ObjectPose.hpp>

class Tracker
{
	public:
		enum State { OFF, INIT_PARAMS_SET, READY_TO_BOOTSTRAP, READY_TO_TRACK };

		virtual ~Tracker() = default;
		virtual void setCameraProperties(const sensor_msgs::msg::CameraInfo& cameraInfo) = 0;
		// Processes a new image. Sets pose.accuracy to -1 if the pose was not updated.
		virtual void updateImage(const cv::Mat& image, ObjectPose& pose) = 0;
		virtual void requestNewTrack() = 0;
		virtual void displayText(const std::vector<std::string>& textLines) = 0;
};

using TrackerPtr = std::shared_ptr<Tracker>;

#endif /* DEXTERITY_TRACKER_HPP_ */
