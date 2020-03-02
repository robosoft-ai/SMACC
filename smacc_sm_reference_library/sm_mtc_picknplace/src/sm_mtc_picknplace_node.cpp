#include <sm_mtc_picknplace/sm_mtc_picknplace.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CalendarWeek");
    smacc::run<sm_mtc_picknplace::SmMTCPickNPlace>();
}
