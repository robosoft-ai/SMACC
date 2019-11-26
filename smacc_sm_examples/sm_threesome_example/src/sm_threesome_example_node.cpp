#include <sm_threesome_example/sm_threesome.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "threesome");
    smacc::run<sm_threesome_example::SmThreeSome>();
}
