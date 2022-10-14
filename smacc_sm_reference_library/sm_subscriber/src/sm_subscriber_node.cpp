#include <sm_subscriber/sm_subscriber.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "SmSubscriber");
    smacc::run<sm_subscriber::SmSubscriber>();
}
