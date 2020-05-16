#include <sm_opencv_3/sm_opencv_3.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_opencv_3");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<sm_opencv_3::SmOpenCV3>();
}