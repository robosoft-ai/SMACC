/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_asynchronous_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

#include <boost/optional.hpp>
#include <tf/transform_listener.h>
namespace cl_move_base_z
{
    class CbMoveBaseClientBehaviorBase : public smacc::SmaccAsyncClientBehavior
    {
    public:
        template <typename TOrthogonal, typename TSourceObject>
        void onOrthogonalAllocation()
        {
            smacc::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
            this->requiresClient(moveBaseClient_);
            moveBaseClient_->onSucceeded(&CbMoveBaseClientBehaviorBase::propagateSuccessEvent, this);
            moveBaseClient_->onAborted(&CbMoveBaseClientBehaviorBase::propagateFailureEvent, this);
        }

    protected:
        ClMoveBaseZ *moveBaseClient_;

    private:
        void propagateSuccessEvent(ClMoveBaseZ::ResultConstPtr& r)
        {
            this->postSuccessEvent();
        }
        void propagateFailureEvent(ClMoveBaseZ::ResultConstPtr& r)
        {
            this->postFailureEvent();
        }
    };
} // namespace cl_move_base_z
