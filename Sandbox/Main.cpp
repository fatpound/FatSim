#include "DroneRouter.hpp"

int main()
{
    fatsim::DroneRouter
    {
        {
            { 534,   272,  150},
            {   0,     0,  150},
            { 200, -1600,  150},
            {1520, -4323, 1900},
            { 199, -4800,  150}
        }
    }
    .Run(2);

    return 0;
}