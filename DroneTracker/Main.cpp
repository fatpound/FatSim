#include "DroneTracker.hpp"

int main()
{
	using enum msr::airlib::ImageCaptureBase::ImageType;

	fatsim::DroneTracker({ { "alpsword", Segmentation, false, false } }, true).Run();

	return 0;
}