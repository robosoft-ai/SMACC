#include <sm_hello_world_example/sm_hello_world.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_world");
    smacc::run<hello_world_example::SmHelloWorld>();
}
