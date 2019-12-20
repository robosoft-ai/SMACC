#pragma once

#include <smacc_navigation_plugin/move_base_action_client.h>
#include <smacc_navigation_plugin/waypoints_navigator_component.h>

namespace smacc
{

template <typename TDerived, typename TObjectTag>
void WaypointNavigator::assignToOrthogonal(ClMoveBaseZ *client)
{
    client_ = client;
    waypointsEventDispatcher.initialize<TDerived, TObjectTag>(client);
}

template <typename TDerived, typename TObjectTag>
void ClMoveBaseZ::assignToOrthogonal()
{
    SmaccActionClientBase<move_base_msgs::MoveBaseAction>::assignToOrthogonal<TDerived, TObjectTag>();
    waypointsNavigator_->template assignToOrthogonal<TDerived, TObjectTag>(this);
}

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