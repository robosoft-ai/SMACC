#include <sm_starcraft_ai/sm_starcraft_ai.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_some");
    smacc::run<sm_starcraft_ai::SmStarcraftAI>();
}
