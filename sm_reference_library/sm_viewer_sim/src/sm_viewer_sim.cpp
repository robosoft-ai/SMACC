#include <sm_viewer_sim/sm_viewer_sim.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_viewer_sim");
    ros::NodeHandle nh;

    smacc::run<sm_viewer_sim::SmViewerSim>();
}
