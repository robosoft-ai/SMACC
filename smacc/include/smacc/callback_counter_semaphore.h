/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <iostream>
#include <boost/signals2.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <boost/signals2.hpp>
#include <ros/ros.h>

namespace smacc
{
class CallbackCounterSemaphore {
public:
    CallbackCounterSemaphore(std::string name, int count = 0);
    bool acquire();

    void release();

    void finalize();

    void addConnection(boost::signals2::connection conn);

private:
    int count_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<boost::signals2::connection> connections_;
    bool finalized = false;
    std::string name_;
};
}
