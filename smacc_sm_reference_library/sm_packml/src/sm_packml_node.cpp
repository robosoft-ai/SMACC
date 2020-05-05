#include <sm_packml/sm_packml.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "packML");
    smacc::run<sm_packml::SmPackML>();
}
