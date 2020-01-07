#include <ros/ros.h>
#include <smacc/smacc.h>
//#include <smacc/smacc_introspection.h>

// ORTHOGONALS
#include <sm_dance_bot_2/orthogonals/or_navigation.h>
#include <sm_dance_bot_2/orthogonals/or_obstacle_perception.h>

// CLIENT BEHAVIORS
#include <move_base_z_client_plugin/client_behaviors/cb_navigate_backward.h>
#include <move_base_z_client_plugin/client_behaviors/cb_navigate_forward.h>
#include <move_base_z_client_plugin/client_behaviors/cb_navigate_global_position.h>
#include <move_base_z_client_plugin/client_behaviors/cb_rotate.h>
#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards.h>

#include <sm_dance_bot_2/clients/move_base_z/client_behaviors/cb_absolute_rotate.h>

using namespace move_base_z_client;

using namespace boost;
using namespace smacc;

namespace sm_dance_bot_2
{

namespace SS1
{
class SsRadialPattern1;
}

/// \brief Advanced example of state machine with smacc that shows multiple
/// techniques
///  for the development of state machines
struct SmDanceBot2
    : public smacc::SmaccStateMachineBase<SmDanceBot2, SS1::SsRadialPattern1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrNavigation>();
    this->createOrthogonal<OrObstaclePerception>();
  }
};

} // namespace sm_dance_bot_2

// SUPERSTATES
#include <sm_dance_bot_2/superstates/ss_radial_pattern_1.h>