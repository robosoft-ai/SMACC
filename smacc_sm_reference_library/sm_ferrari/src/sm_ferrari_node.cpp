#include <sm_ferrari/sm_ferrari.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_some");
    smacc::run<sm_ferrari::SmFerrari>();
}
