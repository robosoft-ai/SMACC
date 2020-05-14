#include <sm_respira_1/sm_respira_1.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "respira_1");
    smacc::run<sm_respira_1::SmRespira1>();
}
