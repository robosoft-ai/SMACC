#include <sm_starcraft_ai/sm_starcraft_ai.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starcraft_ai");
    smacc::run<sm_starcraft_ai::SmStarcraftAI>();
}
