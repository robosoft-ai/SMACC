#include "smacc/common.h"
#include "plugins/smacc_action_client_base.h"
namespace smacc
{
    actionlib::SimpleClientGoalState EvActionClientSuccess::getResult() const
    {
        return client->getState();
    }
}