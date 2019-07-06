#include <radial_motion.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <thread>

//--------------------------------------------
namespace RotateDegress 
{
    struct ToolSubstate: SmaccState<ToolSubstate, ToolOrthogonalLine> 
{  
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    ROS_INFO("Entering ToolSubstate");
  }
};
}