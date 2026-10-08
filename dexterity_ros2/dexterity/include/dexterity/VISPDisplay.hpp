/*
 * VISPDisplay.hpp
 */

#ifndef DEXTERITY_VISPDISPLAY_HPP_
#define DEXTERITY_VISPDISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// X11 window showing the camera image with the tracked object and status text
class VISPDisplay
{
	public:
		explicit VISPDisplay(bool enabled) : enabled(enabled) {}
		void displayImage(vpImage<vpRGBa>& image);
		void displayText(vpImage<vpRGBa>& image, const std::vector<std::string>& textLines);

	private:
		bool enabled;
		std::unique_ptr<vpDisplayX> display;
};

#endif /* DEXTERITY_VISPDISPLAY_HPP_ */
