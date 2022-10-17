/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <chrono>
#include <boost/optional.hpp>
#include <ros/duration.h>
#include <ros/time.h>

namespace smacc
{
class ISmaccUpdatable
{
public:
    ISmaccUpdatable();
    ISmaccUpdatable(ros::Duration duration);

    void executeUpdate();
    void setUpdatePeriod(ros::Duration duration);

protected:
    virtual void update() = 0;

private:
    boost::optional<ros::Duration> periodDuration_;
    ros::Time lastUpdate_;
};
} // namespace smacc
