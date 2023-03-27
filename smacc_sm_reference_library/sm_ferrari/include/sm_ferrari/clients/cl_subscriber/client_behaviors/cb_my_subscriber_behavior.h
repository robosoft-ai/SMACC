
#pragma once
#include <sm_ferrari/clients/cl_subscriber/cl_subscriber.h>
#include <memory>

namespace sm_ferrari
{
namespace cl_subscriber
{

template <typename TSource, typename TOrthogonal>
struct EvMyBehavior: sc::event<EvMyBehavior<TSource, TOrthogonal>>
{

};

class CbMySubscriberBehavior : public smacc::SmaccClientBehavior, public std::enable_shared_from_this<CbMySubscriberBehavior>
{
public:
    void onEntry()
    {
        std::weak_ptr<ClSubscriber> client = this->requiresClient<ClSubscriber>();

        if(auto locked_client = client.lock())
        {
            // locked_client->onMessageReceived(&CbMySubscriberBehavior::onMessageReceived, this);
            //locked_client->onMessageReceived([&](const std_msgs::Float32& msg) -> void 
            //{
            //    if(msg.data > 30)
            //    {
            //        ROS_INFO("[CbMySubscriberBehavior] received message from topic with value > 30. Posting event!");
            //        this->postMyEvent_();
            //    }
            //}, shared_from_this());
            locked_client->onMessageReceived(&CbMySubscriberBehavior::onMessageReceived, shared_from_this());
        }
    }

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
        postMyEvent_ = [=]{this->postEvent<EvMyBehavior<TSourceObject, TOrthogonal>>();};
    }

    void onMessageReceived(const std_msgs::Float32& msg)
    {
        if(msg.data > 30)
        {
            ROS_INFO("[CbMySubscriberBehavior] received message from topic with value > 30. Posting event!");
            this->postMyEvent_();
        }
    }

    std::function<void()> postMyEvent_;
};
} // namespace cl_subscriber
} // namespace sm_ferrari
