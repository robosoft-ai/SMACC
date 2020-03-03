#include <sm_mtc_pour/sm_mtc_pour.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MTCPour");
    smacc::run<sm_mtc_pour::SmMTCPour>();
}
