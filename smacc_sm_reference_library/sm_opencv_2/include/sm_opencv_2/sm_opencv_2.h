#include <smacc/smacc.h>

// CLIENT BEHAVIORS
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/client_behaviors.h>


using namespace cl_move_base_z;

// ORTHOGONALS
#include <sm_opencv_2/orthogonals/or_navigation.h>
#include <sm_opencv_2/orthogonals/or_perception.h>

using namespace smacc;
using namespace smacc::default_events;

namespace sm_opencv_2
{
    class StNavigateToWaypointX;

struct SmOpenCV2
    : public smacc::SmaccStateMachineBase<SmOpenCV2, StNavigateToWaypointX>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrNavigation>();
        this->createOrthogonal<OrPerception>();
    }
};

} // namespace sm_opencv_2


//STATES
#include <sm_opencv_2/states/st_detect_items.h>
#include <sm_opencv_2/states/st_navigate_to_waypoint_x.h>

