#pragma once

#include "smacc/smacc_action_client_base.h"
#include <non_rt_helper/DispenseModeAction.h>

namespace smacc
{
class SmaccReelActionClient: public SmaccActionClientBase<non_rt_helper::DispenseModeAction>
{
    typedef SmaccActionClientBase<non_rt_helper::DispenseModeAction> Base;

    public:
        SmaccReelActionClient();
        SmaccReelActionClient(std::string action_server_namespace);
        virtual std::string getName() const override;
        virtual ~SmaccReelActionClient();
};
}