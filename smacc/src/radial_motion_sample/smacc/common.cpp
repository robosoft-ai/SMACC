#include "smacc/common.h"

namespace smacc
{
    actionlib::SimpleClientGoalState EvActionClientSuccess::getResult()
    {
    return client->getState();
    }
}