#include "DroneRouter.hpp"

int main()
{
    fatsim::DroneRouter{ { {534, 272, 150}, {0, 0, 150}, {200, -1600, 100} } }.Run(5);

    return 0;
}