#include "DroneRouter.hpp"

int main()
{
    fatsim::DroneRouter drouter
    {
        {
           { 200,     0,  150 },
           { 200, -1600,  150 },
           { 200, -4323, 1900 },
           { 200, -4800,  150 }
        },
        5U
    };

    return 0;
}