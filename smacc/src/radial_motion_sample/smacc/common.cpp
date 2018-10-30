#include "smacc/common.h"
#include "smacc/smacc_action_client_base.h"

namespace smacc
{
    actionlib::SimpleClientGoalState EvActionResult::getResult() const
    {
        return client->getState();
    }
}