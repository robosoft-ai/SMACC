/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <iostream>
#include <boost/signals2.hpp>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <boost/signals2.hpp>
#include <ros/ros.h>
#include <smacc/callback_counter_semaphore.h>

namespace smacc
{
    CallbackCounterSemaphore::CallbackCounterSemaphore(std::string name, int count) : count_(count), name_(name) {}

    bool CallbackCounterSemaphore::acquire() {
        std::unique_lock<std::mutex> lock(mutex_);
        ROS_DEBUG("[CallbackCounterSemaphore] acquire callback %s %ld",name_.c_str(), (long)this);

        if(finalized)
        {
            ROS_DEBUG("[CallbackCounterSemaphore] callback rejected %s %ld",name_.c_str(), (long)this);
            return false;
        }

        ++count_;
        cv_.notify_one();

        ROS_DEBUG("[CallbackCounterSemaphore] callback accepted %s %ld",name_.c_str(), (long)this);
        return true;
    }

    void CallbackCounterSemaphore::release() {
        std::unique_lock<std::mutex> lock(mutex_);
        --count_;
        cv_.notify_one();

        ROS_DEBUG("[CallbackCounterSemaphore] callback finished %s %ld",name_.c_str(), (long)this);
    }

    void CallbackCounterSemaphore::finalize() {
        std::unique_lock<std::mutex> lock(mutex_);

        while (count_ > 0) {
            cv_.wait(lock);
        }
        finalized = true;

        for(auto conn: connections_)
        {
            conn.disconnect();
        }

        connections_.clear();
        ROS_DEBUG("[CallbackCounterSemaphore] callbacks finalized %s %ld",name_.c_str(), (long)this);
    }

    void CallbackCounterSemaphore::addConnection(boost::signals2::connection conn)
    {
        std::unique_lock<std::mutex> lock(mutex_);

        if(finalized)
        {
            ROS_DEBUG("[CallbackCounterSemaphore] ignoring adding callback, already finalized %s %ld",name_.c_str(), (long)this);
            return;
        }

        connections_.push_back(conn);
    }

}
