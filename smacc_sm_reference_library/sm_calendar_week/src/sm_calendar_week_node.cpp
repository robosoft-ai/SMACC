#include <sm_calendar_week/sm_calendar_week.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CalendarWeek");
    smacc::run<sm_calendar_week::SmCalendarWeek>();
}
