#include <sm_pr2_plugs/sm_pr2_plugs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PR2Plugs");
    smacc::run<sm_pr2_plugs::SmPR2Plugs>();
}
