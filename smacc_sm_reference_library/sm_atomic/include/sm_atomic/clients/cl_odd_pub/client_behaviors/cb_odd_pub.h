#pragma once

#include <sm_atomic/clients/cl_odd_pub/cl_odd_pub.h>
#include <smacc/smacc_client_behavior.h>
#include <std_msgs/String.h>

namespace sm_atomic
{
namespace cl_odd_pub
{
class CbOddPub : public smacc::SmaccClientBehavior
{
public:
    ClOddPub *publisherClient_;
    std::string msg_;

    CbOddPub(std::string msg)
    {
        ROS_INFO_STREAM("Creating CbOddPub behavior with stored message: " << msg);
        msg_ = msg;
    }

    virtual void onEntry()
    {
        this->requiresClient(publisherClient_);
        // std_msgs::String rosmsg;
        // rosmsg.data = msg_;
        // publisherClient_->publish(rosmsg);
    }

    virtual void onExit() override
    {
        std_msgs::String rosmsg;
        rosmsg.data = msg_;
        publisherClient_->publish(rosmsg);
    }
};
} // namespace cl_odd_pub
} // namespace sm_atomic
