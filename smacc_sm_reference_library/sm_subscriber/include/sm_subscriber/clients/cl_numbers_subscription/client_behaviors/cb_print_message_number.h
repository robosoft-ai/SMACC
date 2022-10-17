#pragma once

#include <sm_subscriber/clients/cl_numbers_subscription/cl_numbers_subscription.h>
#include <smacc/client_behavior_bases/cb_subscription_callback_base.h>

namespace sm_subscriber
{

class CbPrintMessageNumber : public smacc::CbSubscriptionCallbackBase<std_msgs::UInt16>
{
public:
  void onMessageReceived(const std_msgs::UInt16& msg) override
  {
    ROS_WARN_STREAM("**** [CbPrintMessageNumber] MESSAGE RECEIVED NUMBER: " << msg.data);
  }
};

};  // namespace sm_subscriber
