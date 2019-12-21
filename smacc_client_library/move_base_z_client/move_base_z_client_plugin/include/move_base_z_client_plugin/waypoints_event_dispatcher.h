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

template <typename TSource, typename TObjectTag>
struct EvWaypoint2 : sc::event<EvWaypoint2<TSource, TObjectTag>>
{
    int waypointIndex;
};

template <typename TSource, typename TObjectTag>
struct EvWaypoint3 : sc::event<EvWaypoint3<TSource, TObjectTag>>
{
    int waypointIndex;
};

template <typename TSource, typename TObjectTag>
struct EvWaypoint4 : sc::event<EvWaypoint4<TSource, TObjectTag>>
{
    int waypointIndex;
};

class ClMoveBaseZ;

#define WAYPOINTS_EVENTCOUNT 1000

class WaypointEventDispatcher
{
    std::function<void()> postWaypointFn[WAYPOINTS_EVENTCOUNT];

public:
    template <typename TDerived, typename TObjectTag>
    void initialize(ClMoveBaseZ *client);

    void postWaypointEvent(int index);
};
} // namespace smacc