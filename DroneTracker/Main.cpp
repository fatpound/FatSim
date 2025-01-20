#include "DroneTracker.hpp"

int main()
{
	fatsim::DroneTracker droneTracker(
		{ { "alpsword", msr::airlib::ImageCaptureBase::ImageType::Segmentation, false, false } },
		true
	);

	droneTracker.Run();

	return 0;
}