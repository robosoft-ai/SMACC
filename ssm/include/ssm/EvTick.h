/** @file EvFailed.h
 *  @brief
 *  @author Bob P
 *  @date 8 Oct 2019
 *  @copyright Copyright 2019 (c) GreyOrange Inc. All rights reserved.
 */
#pragma once
#ifndef __SIMPLE_STATE_MACHINE__EV_TICK_H__
#define __SIMPLE_STATE_MACHINE__EV_TICK_H__

#include <boost/statechart/event.hpp>

namespace simple_state_machine {

/**
 * @brief Event to signal that the current operation has failed.
 *
 */
class EvTick : public boost::statechart::event<EvTick>
{
public:

private:
};

}  // namespace simple_state_machine
#endif  //__SIMPLE_STATE_MACHINE__EV_TICK_H__
