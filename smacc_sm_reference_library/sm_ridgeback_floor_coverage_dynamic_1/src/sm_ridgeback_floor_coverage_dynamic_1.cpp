#include <sm_ridgeback_floor_coverage_dynamic_1/sm_ridgeback_floor_coverage_dynamic_1.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dance_bot_strikes_back");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<SmRidgebackFloorCoverageDynamic1>();
}