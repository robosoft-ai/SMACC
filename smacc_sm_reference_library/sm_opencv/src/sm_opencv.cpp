#include <sm_opencv/sm_opencv.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_opencv");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<sm_opencv::SmOpenCV>();
}