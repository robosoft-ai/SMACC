#include <sm_packML/sm_packML.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "three_some");
    smacc::run<sm_packML::SmThreesome>();
}
