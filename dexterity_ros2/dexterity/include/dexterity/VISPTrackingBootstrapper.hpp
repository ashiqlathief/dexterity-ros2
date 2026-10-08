/*
 * VISPTrackingBootstrapper.hpp
 *
 * Starts (and restarts) the model based tracker from the pose of an AprilTag on the object.
 */

#ifndef DEXTERITY_VISPTRACKINGBOOTSTRAPPER_HPP_
#define DEXTERITY_VISPTRACKINGBOOTSTRAPPER_HPP_

#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

class VISPTrackingBootstrapper
{
	public:
		bool isReady() const { return bootstrapperReady; }
		void init(double tagSizeParam, const std::string& poseEstimationMethodParam, const std::string& tagFamilyParam);
		bool bootstrapTrackingAuto(vpImage<unsigned char>& I_bw, vpImage<vpRGBa>& I_color, vpCameraParameters& camParams, vpMbGenericTracker& tracker);

	private:
		std::unique_ptr<vpDetectorAprilTag> detector;
		double tagSize = 0;
		bool bootstrapperReady = false;
		float quadDecimate = 1.0;
		int nThreads = 1;
		unsigned int thickness = 2;
};

#endif /* DEXTERITY_VISPTRACKINGBOOTSTRAPPER_HPP_ */
