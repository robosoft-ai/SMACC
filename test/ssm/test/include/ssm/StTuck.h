/// Testing stub
#pragma once
#ifndef __SIMPLE_STATE_MACHINE__ST_TUCK_H__
#define __SIMPLE_STATE_MACHINE__ST_TUCK_H__

#include <smacc/smacc.h>

namespace simple_state_machine {

class SimpleStateMachine;

/**
 * @brief A simple state for the Simple State Machine example.
 */
class  StTuck: public smacc::SmaccState<StTuck, SimpleStateMachine>
{
public:
    using SmaccState::SmaccState;

    // Transitions
    typedef boost::mpl::list<
    > reactions;

    void onInitialize()
    {
        ROS_INFO("StTuck::onInitialize: testing stub");
    }

    void onEntry()
    {
        std::string message = "foo";
        getStateMachine().getGlobalSMData("message", message);
        ROS_INFO_STREAM("***** " << "StTuck testing stub: " << message << " *****");
    }

private:
};

}  // namespace simple_state_machine

#endif  //__SIMPLE_STATE_MACHINE__ST_TUCK_H__
