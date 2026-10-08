/*
 * VISPDisplay.cpp
 */

#include <dexterity/VISPDisplay.hpp>

#include <visp3/core/vpDisplay.h>

void VISPDisplay::displayImage(vpImage<vpRGBa>& image)
{
	if (!enabled)
	{
		return;
	}
	if (!display)
	{
		display = std::make_unique<vpDisplayX>(image, -1, -1, "dexterity - wrist camera");
	}
	vpDisplay::display(image);
}

void VISPDisplay::displayText(vpImage<vpRGBa>& image, const std::vector<std::string>& textLines)
{
	if (!enabled || !display)
	{
		return;
	}
	const int interLineSpace = 15;
	int i = 15;
	for (const auto& line : textLines)
	{
		vpDisplay::displayText(image, i, 10, line, vpColor::red);
		i += interLineSpace;
	}
	vpDisplay::flush(image);
}
