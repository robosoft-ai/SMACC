/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/smacc_client_behavior.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>

#include <boost/optional.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace cl_move_base_z
{
class CbAbsoluteRotate : public smacc::SmaccClientBehavior
{
public:
    enum class SpiningPlanner {Default, PureSpinning, Forward};

    tf::TransformListener listener;

    ClMoveBaseZ *moveBaseClient_;

    boost::optional<float> absoluteGoalAngleDegree;
    boost::optional<float> yawGoalTolerance;
    boost::optional<SpiningPlanner> spinningPlanner;    

    CbAbsoluteRotate();

    CbAbsoluteRotate(float absoluteGoalAngleDegree, float yawGoalTolerance = 0.15);

    virtual void onEntry() override;
    virtual void onExit() override;

    private:
        void setLocalPlannerYawTolerance(float newtolerance);
        float oldYawTolerance;
};
} // namespace cl_move_base_z
