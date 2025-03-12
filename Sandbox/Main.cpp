#include "DroneRouter.hpp"

int main()
{
    fatsim::DroneRouter{ { {534, 272, 150}, {0, 0, 150} } }.Run();

    return 0;
}