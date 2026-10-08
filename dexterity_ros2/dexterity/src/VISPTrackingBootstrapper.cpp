/*
 * VISPTrackingBootstrapper.cpp
 */

#include <dexterity/VISPTrackingBootstrapper.hpp>

#include <map>

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <visp3/core/vpDisplay.h>

namespace
{
const rclcpp::Logger logger = rclcpp::get_logger("dexterity.visp");
rclcpp::Clock throttleClock(RCL_STEADY_TIME);
}

void VISPTrackingBootstrapper::init(double tagSizeParam, const std::string& poseEstimationMethodParam, const std::string& tagFamilyParam)
{
	static const std::map<std::string, vpDetectorAprilTag::vpPoseEstimationMethod> poseEstimationMethods {
		{"homography", vpDetectorAprilTag::HOMOGRAPHY},
		{"homography_virtual_vs", vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS},
		{"dementhon_virtual_vs", vpDetectorAprilTag::DEMENTHON_VIRTUAL_VS},
		{"lagrange_virtual_vs", vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS},
		{"best_virtual_vs", vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS},
		{"homography_orthogonal_iteration", vpDetectorAprilTag::HOMOGRAPHY_ORTHOGONAL_ITERATION}};
	static const std::map<std::string, vpDetectorAprilTag::vpAprilTagFamily> tagFamilies {
		{"36h11", vpDetectorAprilTag::TAG_36h11},
		{"25h9", vpDetectorAprilTag::TAG_25h9},
		{"16h5", vpDetectorAprilTag::TAG_16h5},
		{"standard41h12", vpDetectorAprilTag::TAG_STANDARD41h12}};

	tagSize = tagSizeParam;
	auto method = poseEstimationMethods.find(poseEstimationMethodParam);
	auto family = tagFamilies.find(tagFamilyParam);
	if (tagSize <= 0 || method == poseEstimationMethods.end() || family == tagFamilies.end())
	{
		bootstrapperReady = false;
		RCLCPP_ERROR(logger, "Invalid AprilTag settings: size %.3f, family '%s', pose estimation method '%s'",
			tagSize, tagFamilyParam.c_str(), poseEstimationMethodParam.c_str());
		return;
	}

	detector = std::make_unique<vpDetectorAprilTag>(family->second);
	detector->setAprilTagQuadDecimate(quadDecimate);
	detector->setAprilTagPoseEstimationMethod(method->second);
	detector->setAprilTagNbThreads(nThreads);
	detector->setDisplayTag(true, vpColor::red, thickness);
	bootstrapperReady = true;
	RCLCPP_INFO(logger, "AprilTag bootstrapper ready (family %s, size %.3f m)", tagFamilyParam.c_str(), tagSize);
}

bool VISPTrackingBootstrapper::bootstrapTrackingAuto(vpImage<unsigned char>& I_bw, vpImage<vpRGBa>& I_color, vpCameraParameters& camParams, vpMbGenericTracker& tracker)
{
	if (!bootstrapperReady)
	{
		return false;
	}
	std::vector<vpHomogeneousMatrix> cMo_vec;
	bool success = detector->detect(I_bw, tagSize, camParams, cMo_vec);
	if (!success || cMo_vec.empty())
	{
		RCLCPP_INFO_THROTTLE(logger, throttleClock, 2000, "No AprilTag in the current image, trying again...");
		return false;
	}

	vpDisplay::displayFrame(I_color, cMo_vec.back(), camParams, 0.025, vpColor::blue, 3);
	auto t = cMo_vec.back().getTranslationVector();
	RCLCPP_INFO(logger, "AprilTag found at (%.3f, %.3f, %.3f), initializing the model based tracker", t[0], t[1], t[2]);

	// The CAD model is defined in the tag frame, so the tag pose initializes the object pose
	tracker.initFromPose(I_color, cMo_vec.back());
	tracker.setProjectionErrorComputation(true);
	tracker.setAngleAppear(vpMath::rad(70));
	tracker.setAngleDisappear(vpMath::rad(80));
	tracker.setClipping(tracker.getClipping() | vpPolygon3D::FOV_CLIPPING);
	return true;
}
