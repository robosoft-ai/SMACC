#include <sm_ferrari/sm_ferrari.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_ferrari_node");
    smacc::run<sm_ferrari::SmFerrari>();
}
