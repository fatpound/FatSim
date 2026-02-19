#include "DroneTracker.hpp"

int main()
{
    using enum msr::airlib::ImageCaptureBase::ImageType;

    fatsim::DroneTracker(
        {
            { "fatsim_segmentation",      Segmentation,     false, false },
            { "fatsim_depth_perspective", DepthPerspective,  true, false }
        },
        true
    )
    .Run();

    return 0;
}
