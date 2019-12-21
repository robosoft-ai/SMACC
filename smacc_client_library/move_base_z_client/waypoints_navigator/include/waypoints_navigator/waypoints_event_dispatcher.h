#pragma once

#include <smacc/smacc.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

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

template <typename TEv>
void configurePostEvWaypoint(std::function<void()> *fntarget, ClMoveBaseZ *client, int index)
{
    fntarget[index] = [=]() {
        client->template postEvent<TEv>();
    };
}

template <typename TDerived, typename TObjectTag>
void WaypointEventDispatcher::initialize(ClMoveBaseZ *client)
{
    configurePostEvWaypoint<EvWaypoint0<TDerived, TObjectTag>>(postWaypointFn, client, 0);
    configurePostEvWaypoint<EvWaypoint1<TDerived, TObjectTag>>(postWaypointFn, client, 1);
    configurePostEvWaypoint<EvWaypoint2<TDerived, TObjectTag>>(postWaypointFn, client, 2);
    configurePostEvWaypoint<EvWaypoint3<TDerived, TObjectTag>>(postWaypointFn, client, 3);
    configurePostEvWaypoint<EvWaypoint4<TDerived, TObjectTag>>(postWaypointFn, client, 4);

    // postWaypointFn[0] = [=]() {
    //     client->template postEvent<EvWaypoint0<TDerived, TObjectTag>>();
    // };

    // postWaypointFn[1] = [=]() {
    //     client->template postEvent<EvWaypoint1<TDerived, TObjectTag>>();
    // };
}

} // namespace smacc