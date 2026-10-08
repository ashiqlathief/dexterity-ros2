/*
 * VISPTracker.cpp
 */

#include <dexterity/VISPTracker.hpp>

#include <filesystem>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpQuaternionVector.h>

namespace
{
const rclcpp::Logger logger = rclcpp::get_logger("dexterity.visp");
}

VISPTracker::VISPTracker(const PerceptionSettings& settings, const std::string& cameraOpticalFrame)
	: cameraOpticalFrame(cameraOpticalFrame), vispDisplay(settings.showTrackingWindow)
{
	if (settings.trackerMode == "me")
	{
		mode = TM_MOVING_EDGES;
	}
	else if (settings.trackerMode == "klt")
	{
		mode = TM_KLT;
	}
	else if (settings.trackerMode == "hybrid")
	{
		mode = TM_HYBRID;
	}
	else
	{
		throw std::runtime_error("ViSP only supports the tracker modes me, klt and hybrid, got: " + settings.trackerMode);
	}
	setTrackerMode(mode);
	loadModel(settings.objectModelFilename);
	tracker.setDisplayFeatures(true);
	bootstrapper.init(settings.tagSize, settings.poseEstimationMethod, settings.tagFamily);
	state = INIT_PARAMS_SET;
}

void VISPTracker::loadModel(const std::string& filename)
{
	std::filesystem::path path(filename);
	if (path.is_relative())
	{
		path = std::filesystem::path(ament_index_cpp::get_package_share_directory("dexterity")) / "config" / "cad_models" / filename;
	}
	try
	{
		tracker.loadModel(path.string());
	}
	catch (const vpException& e)
	{
		throw std::runtime_error("Failed to load the object model " + path.string() + ": " + e.getMessage());
	}
	RCLCPP_INFO(logger, "Object model loaded: %s", path.c_str());
}

void VISPTracker::setCameraProperties(const sensor_msgs::msg::CameraInfo& cameraInfo)
{
	camParams.initPersProjWithoutDistortion(cameraInfo.k[0], cameraInfo.k[4], cameraInfo.k[2], cameraInfo.k[5]);
	tracker.setCameraParameters(camParams);
	camParamsSet = true;
}

void VISPTracker::setTrackerMode(Mode newMode)
{
	mode = newMode;
	if (mode == TM_MOVING_EDGES)
	{
		tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
	}
	else if (mode == TM_KLT)
	{
		tracker.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
	}
	else
	{
		tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
	}

	if (mode == TM_MOVING_EDGES || mode == TM_HYBRID)
	{
		vpMe me;
		me.setMaskSize(5);
		me.setMaskNumber(180);
		me.setRange(8);
		me.setLikelihoodThresholdType(vpMe::OLD_THRESHOLD); // Same threshold semantics as the ViSP version used in ROS 1
		me.setThreshold(10000);
		me.setMu1(0.5);
		me.setMu2(0.5);
		me.setSampleStep(4);
		tracker.setMovingEdge(me);
	}
	if (mode == TM_KLT || mode == TM_HYBRID)
	{
		vpKltOpencv kltSettings;
		kltSettings.setMaxFeatures(300);
		kltSettings.setWindowSize(5);
		kltSettings.setQuality(0.015);
		kltSettings.setMinDistance(8);
		kltSettings.setHarrisFreeParameter(0.01);
		kltSettings.setBlockSize(3);
		kltSettings.setPyramidLevels(3);
		tracker.setKltOpencv(kltSettings);
		tracker.setKltMaskBorder(5);
	}
}

void VISPTracker::updateImage(const cv::Mat& image, ObjectPose& pose)
{
	pose.poseValidity = false;
	pose.accuracy = -1;
	vpImageConvert::convert(image, I_color);
	vpImageConvert::convert(image, I_bw);
	vispDisplay.displayImage(I_color);

	if (state == INIT_PARAMS_SET)
	{
		if (!camParamsSet)
		{
			return;
		}
		state = READY_TO_BOOTSTRAP;
	}
	if (state == READY_TO_BOOTSTRAP)
	{
		if (bootstrapper.bootstrapTrackingAuto(I_bw, I_color, camParams, tracker))
		{
			state = READY_TO_TRACK;
		}
	}
	if (state == READY_TO_TRACK)
	{
		pose = calculateObjectPose();
	}
}

ObjectPose VISPTracker::calculateObjectPose()
{
	ObjectPose objectPose;
	vpHomogeneousMatrix cMo;
	try
	{
		tracker.track(I_color);
		tracker.getPose(cMo);
		tracker.display(I_color, cMo, camParams, vpColor::red, 2);
		objectPose.poseStamped = computePoseStamped(cMo);
		double projectionError = computeProjectionError(cMo);
		objectPose.poseValidity = true;
		objectPose.accuracy = ((90 - projectionError) / 90) * 100;
	}
	catch (const vpException& e)
	{
		RCLCPP_WARN(logger, "Tracking lost: %s", e.getMessage());
		objectPose.poseValidity = false;
		objectPose.accuracy = -1; // Tells the perception module that the pose was not updated
		state = READY_TO_BOOTSTRAP;
	}
	return objectPose;
}

double VISPTracker::computeProjectionError(vpHomogeneousMatrix& cMo)
{
	if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER)
	{
		return tracker.getProjectionError();
	}
	return tracker.computeCurrentProjectionError(I_color, cMo, camParams);
}

geometry_msgs::msg::PoseStamped VISPTracker::computePoseStamped(vpHomogeneousMatrix& cMo)
{
	geometry_msgs::msg::PoseStamped pose;
	vpTranslationVector translation = cMo.getTranslationVector();
	vpQuaternionVector q;
	cMo.extract(q);
	pose.header.frame_id = cameraOpticalFrame;
	pose.pose.position.x = translation[0];
	pose.pose.position.y = translation[1];
	pose.pose.position.z = translation[2];
	tf2::Quaternion tq {q.x(), q.y(), q.z(), q.w()};
	pose.pose.orientation = tf2::toMsg(tq.normalized());
	return pose;
}

void VISPTracker::displayText(const std::vector<std::string>& textLines)
{
	vispDisplay.displayText(I_color, textLines);
}
