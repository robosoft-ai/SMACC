#pragma once

#include <smacc/smacc.h>

namespace smacc
{

template <typename TSource, typename TObjectTag>
struct EvWaypoint0 : sc::event<EvWaypoint0<TSource, TObjectTag>>
{
    int waypointIndex;
};

template <typename TSource, typename TObjectTag>
struct EvWaypoint1 : sc::event<EvWaypoint1<TSource, TObjectTag>>
{
    int waypointIndex;
};

class SmaccMoveBaseActionClient;

#define WAYPOINTS_EVENTCOUNT 1000

class WaypointEventDispatcher
{
    std::function<void()> postWaypointFn[WAYPOINTS_EVENTCOUNT];

public:
    template <typename TDerived, typename TObjectTag>
    void initialize(SmaccMoveBaseActionClient *client);

    void postWaypointEvent(int index);
};
} // namespace smacc