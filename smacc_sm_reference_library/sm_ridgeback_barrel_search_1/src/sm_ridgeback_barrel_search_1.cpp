#include <sm_ridgeback_barrel_search_1/sm_ridgeback_barrel_search_1.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_ridgeback_barrel_search_1");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<sm_ridgeback_barrel_search_1::SmRidgebackBarrelSearch1>();
}