#include <sm_packML/sm_packML.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "packML");
    smacc::run<sm_packML::SmPackML>();
}
