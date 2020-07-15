/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <smacc/smacc_client_behavior_base.h>
#include <thread>

namespace smacc
{
    class SmaccAsyncClientBehavior : public ISmaccClientBehavior
    {
    protected:
        virtual ~SmaccAsyncClientBehavior();
        virtual void executeOnEntry() override;
        virtual void executeOnExit() override;
    private:
        std::thread onEntryThread_;
        std::thread onExitThread_;
    };
} // namespace smacc

#include <smacc/impl/smacc_client_behavior_impl.h>