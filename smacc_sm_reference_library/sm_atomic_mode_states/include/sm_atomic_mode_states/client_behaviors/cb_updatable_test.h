#pragma once

#include <smacc/smacc.h>

class CbUpdatableTest : public smacc::ISmaccUpdatable, public smacc::ISmaccClientBehavior
{
public:
    CbUpdatableTest()
    {
        this->counter_ = 0;
    }

    virtual void onEntry()
    {
        ROS_INFO("CbUpdatableTest::onEntry");
    }

    virtual void onExit()
    {
        ROS_INFO("CbUpdatableTest::onExit");
    }

    virtual void update() override
    {
        auto currentState = this->getCurrentState();
        this->counter_++;
        ROS_INFO("CbUpdatableTest::onUpdate %d in state %s", this->counter_, currentState->getClassName().c_str());
    }

    int counter_;
};
