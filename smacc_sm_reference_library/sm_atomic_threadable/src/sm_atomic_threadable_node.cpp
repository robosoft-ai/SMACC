#include <sm_atomic_threadable/sm_atomic_threadable.h>

//--------------------------------------------------------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "sm_atomic_threadable");
  ros::NodeHandle nh;

  smacc::run<sm_atomic_threadable::SmAtomicThreadable>();

  return 0;
}
