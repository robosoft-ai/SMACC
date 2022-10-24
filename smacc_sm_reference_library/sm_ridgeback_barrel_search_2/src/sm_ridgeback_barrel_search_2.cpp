#include <sm_ridgeback_barrel_search_2/sm_ridgeback_barrel_search_2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_ridgeback_barrel_search_2");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<sm_ridgeback_barrel_search_2::SmRidgebackBarrelSearch2>();
}
