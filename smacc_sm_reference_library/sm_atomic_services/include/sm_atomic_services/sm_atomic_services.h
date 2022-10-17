#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>
#include "sm_atomic_services/clients/cl_service_server.h"
#include "sm_atomic_services/clients/cl_service_client.h"

// ORTHOGONALS
#include <sm_atomic_services/orthogonals/or_timer.h>
#include <sm_atomic_services/orthogonals/or_services.h>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.h>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>
#include "sm_atomic_services/clients/client_behaviors/cb_service_server.h"

using namespace boost;
using namespace smacc;

namespace sm_atomic_services
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicServices
    : public smacc::SmaccStateMachineBase<SmAtomicServices, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrServices>();
    }
};

} // namespace sm_atomic

#include <sm_atomic_services/states/st_state_1.h>
#include <sm_atomic_services/states/st_state_2.h>
