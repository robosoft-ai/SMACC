
#include <sm_subscriber/clients/cl_numbers_subscription/cl_numbers_subscription.h>
#include <smacc/client_behavior_bases/cb_subscription_callback_base.h>

namespace sm_subscriber
{
template <typename TSourceObject, typename Orthogonal>
struct EvImportantMessage : sc::event<EvImportantMessage<TSourceObject, Orthogonal>>
{
  int value;
};

class CbPostCustomEvent : public smacc::CbSubscriptionCallbackBase<std_msgs::UInt16>
{
public:
  void onMessageReceived(const std_msgs::UInt16& msg) override
  {
    if (msg.data % 4 == 0)
    {
      ROS_WARN_STREAM("**** [CbPostCustomEvent] MESSAGE RECEIVED NUMBER MULTIPLE OF 4 "<< msg.data <<" , POSTING EvImportantMessage");
      this->postEventImportantMessage(msg.data);
    }
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postEventImportantMessage = [=](int value) {
      auto ev = new EvImportantMessage<TSourceObject, TOrthogonal>();
      ev->value =  value;
      this->postEvent(ev);
    };
  }

  std::function<void(int)> postEventImportantMessage;
};

};  // namespace sm_subscriber
