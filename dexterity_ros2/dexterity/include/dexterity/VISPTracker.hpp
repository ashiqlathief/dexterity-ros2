/*
 * VISPTracker.hpp
 *
 * ViSP model based tracker (moving edges, KLT or hybrid) bootstrapped with an AprilTag.
 */

#ifndef DEXTERITY_VISPTRACKER_HPP_
#define DEXTERITY_VISPTRACKER_HPP_

#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <dexterity/Settings.hpp>
#include <dexterity/Tracker.hpp>
#include <dexterity/VISPDisplay.hpp>
#include <dexterity/VISPTrackingBootstrapper.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

class VISPTracker : public Tracker
{
	public:
		enum Mode { TM_MOVING_EDGES, TM_KLT, TM_HYBRID };

		VISPTracker(const PerceptionSettings& settings, const std::string& cameraOpticalFrame);
		void setCameraProperties(const sensor_msgs::msg::CameraInfo& cameraInfo) override;
		void updateImage(const cv::Mat& image, ObjectPose& pose) override;
		void requestNewTrack() override { state = READY_TO_BOOTSTRAP; }
		void displayText(const std::vector<std::string>& textLines) override;

	private:
		Mode mode = TM_HYBRID;
		State state = OFF;
		std::string cameraOpticalFrame;
		VISPDisplay vispDisplay;
		vpImage<vpRGBa> I_color;
		vpImage<unsigned char> I_bw;
		vpMbGenericTracker tracker;
		VISPTrackingBootstrapper bootstrapper;
		vpCameraParameters camParams;
		bool camParamsSet = false;

		void setTrackerMode(Mode newMode);
		void loadModel(const std::string& filename);
		ObjectPose calculateObjectPose();
		double computeProjectionError(vpHomogeneousMatrix& cMo);
		geometry_msgs::msg::PoseStamped computePoseStamped(vpHomogeneousMatrix& cMo);
};

#endif /* DEXTERITY_VISPTRACKER_HPP_ */
