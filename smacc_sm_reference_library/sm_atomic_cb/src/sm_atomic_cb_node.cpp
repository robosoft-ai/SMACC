#include <sm_atomic_cb/sm_atomic_cb.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_atomic_cb");
    ros::NodeHandle nh;

    smacc::run<sm_atomic_cb::SmAtomicCB>();
}