#include "smacc/common.h"
#include "smacc/smacc_action_client_base.h"

namespace smacc
{
    actionlib::SimpleClientGoalState IActionResult::getResult() const
    {
        return client->getState();
    }
}