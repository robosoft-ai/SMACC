#include <sm_atomic_http/sm_atomic_http.h>

//--------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_atomic_htt");
    ros::NodeHandle nh;

    smacc::run<sm_atomic_http::SmAtomicHttp>();
}