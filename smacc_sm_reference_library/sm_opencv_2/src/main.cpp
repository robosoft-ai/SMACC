#include <sm_opencv_2/sm_opencv_2.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sm_opencv_2");
    ros::NodeHandle nh;

    ros::Duration(5).sleep();
    smacc::run<sm_opencv_2::SmOpenCV2>();
}