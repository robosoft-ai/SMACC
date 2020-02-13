#include <sm_dance_bot_strikes_back/sm_dance_bot_strikes_back.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dance_bot_strikes_back");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<SmDanceBotStrikesBack>();
}
