#include <sm_three_some/sm_three_some.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_some");
    smacc::run<sm_three_some::SmThreeSome>();
}
