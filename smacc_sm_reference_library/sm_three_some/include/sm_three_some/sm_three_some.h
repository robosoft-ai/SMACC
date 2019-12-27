#include <ros/ros.h>
#include <smacc/smacc.h>

// CLIENTS
#include <sm_three_some/clients/cl_client_1.h>
#include <sm_three_some/clients/cl_client_2.h>
#include <sm_three_some/clients/cl_keyboard.h>

using namespace sm_three_some::client_1;
using namespace sm_three_some::client_2;
using namespace sm_three_some::keyboard_client;

// ORTHOGONALS
#include <sm_three_some/orthogonals/or_orthogonal_1.h>
#include <sm_three_some/orthogonals/or_orthogonal_2.h>
#include <sm_three_some/orthogonals/or_keyboard_orthogonal.h>

//CLIENT BEHAVIORS
#include <sm_three_some/client_behaviors/client_1/client_behavior_1.h>
#include <sm_three_some/client_behaviors/client_1/client_behavior_1b.h>
#include <sm_three_some/client_behaviors/client_2/client_behavior_2.h>
#include <sm_three_some/client_behaviors/client_2/client_behavior_2b.h>
#include <sm_three_some/client_behaviors/keyboard/cb_keyboard_behavior.h>


//LOGIC UNITS
#include <all_events_go/lu_all_events_go.h>

using namespace smacc;

namespace sm_three_some
{
//SUPERSTATES
namespace SS1
{
class Ss1;
}

//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;

// STATE MACHINE
struct SmThreeSome
    : public smacc::SmaccStateMachineBase<SmThreeSome, StState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrOrthogonal1>();
        this->createOrthogonal<OrOrthogonal2>();
        this->createOrthogonal<OrKeyboard>();
    }
};
} // namespace sm_three_some

//STATES
#include <sm_three_some/states/st_state_1.h>
#include <sm_three_some/states/st_state_2.h>
#include <sm_three_some/states/st_state_3.h>

#include <sm_three_some/superstates/ss_superstate_1.h>