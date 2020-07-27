#include <sm_ridgeback_floor_coverage_static_1/sm_ridgeback_floor_coverage_static_1.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dance_bot");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<SmRidgebackFloorCoverageStatic1>();
}
