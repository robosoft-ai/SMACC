#include <sm_threesome/sm_threesome.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "threesome");
    smacc::run<sm_threesome::SmThreeSome>();
}
