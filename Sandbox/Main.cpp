#include "DroneRouter.hpp"

int main()
{
    fatsim::DroneRouter simulation({{0, 5, -14}, {0, 55, -14}});

    simulation.Run();

    return 0;
}