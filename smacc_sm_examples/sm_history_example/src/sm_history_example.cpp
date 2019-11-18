#include <sm_history_example/sm_history_example.h>

int main(int argc, char **argv)
{
//    test_main(argc,argv);

     ros::init(argc, argv, "sm_history_example");
     ros::NodeHandle nh;

     smacc::run<SmHistoryExample>();
}
