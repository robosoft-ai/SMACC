#pragma once

#include <smacc/smacc.h>
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <sensor_msgs/BatteryState.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>

#include <std_msgs/UInt16.h>

namespace cl_battery_monitor
{
class ClBatteryMonitor : public smacc::client_bases::SmaccSubscriberClient<sensor_msgs::BatteryState>
{
public:
    ClBatteryMonitor();
    ClBatteryMonitor(std::string topicname);

    virtual ~ClBatteryMonitor();

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
        smacc::client_bases::SmaccSubscriberClient<sensor_msgs::BatteryState>::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    }
};
}